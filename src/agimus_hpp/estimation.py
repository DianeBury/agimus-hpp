#!/usr/bin/env python
import rospy, hpp.corbaserver
from .client import HppClient
from dynamic_graph_bridge_msgs.msg import Vector
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf import TransformBroadcaster
from std_msgs.msg import Empty, UInt32
from std_srvs.srv import SetBool, SetBoolRequest
from math import cos, sin
from threading import Lock
import traceback
import ros_tools

### \brief Estimation based on HPP constraint solver.
##
## This class solves the following problem.
##
## Given:
## \li the current encoder values,
## \li tag poses (attached to objects) in the robot camera frame,
## \li a set of contraints encoding the semantic,
## estimate the complete robot configuration, including the robot base and, optionally, the object poses.
##
## The semantic of the problem (foot on the ground, object on the table...)
## can be expressed with HPP constraints. The cost is a mixture of:
## \li visual tag constraints: it penalizes the error between the tag position from CV and from estimation.
## \li current robot pose: it penalizes the error between the current robot pose from encoders and from estimation.
##
## There are two ways of specifying the semantic constraints:
## \li Core framework: set the ROS parameter "default_constraints" to a list of constraint names.
## \li Manipulation framework:
##     - the current state of the manipulation graph is estimated using the last configuration in
##       HPP server. It is a mixture of the result of the previous estimation and of the encoder values.
##     - if the current state cannot be estimated, it is assumed it has not changed since last iteration.
##     - the constraint of this state are used for estimation.
##
## Connection with HPP is handled by agimus_hpp.client.HppClient.
class Estimation(HppClient):
    ## Subscribed topics (prefixed by "/agimus")
    subscribersDict = {
            "estimation": {
                "request" : [Empty, "estimation" ],
                },
            "vision": {
                "tags": [TransformStamped, "get_visual_tag"],
                },
            "sot": {
                "base_pose_estimation": [TransformStamped, "get_base_pose_estimation"],
                },
            }
    ## Provided services (prefixed by "/agimus")
    servicesDict = {
            "estimation": {
                "continuous_estimation" : [SetBool, "continuous_estimation" ],
                },
            }
    ## Published topics (prefixed by "/agimus")
    publishersDict = {
            "estimation": {
                # "estimation"          : [ Vector, 1],
                "semantic" : [ Vector, 1],
                "state_id" : [ UInt32, 1],
                },
            }

    def __init__ (self, continuous_estimation = False,
             joint_states_topic="/joint_states",
             visual_tags_enabled=True):
        super(Estimation, self).__init__ (context = "estimation")

        self.locked_joints = []

        self.tf_pub = TransformBroadcaster()
        self.tf_root = "world"

        self.mutex = Lock()

        self.robot_name = rospy.get_param("robot_name", "")

        self.last_stamp_is_ready = False
        self.last_stamp = rospy.Time.now()
        self.last_visual_tag_constraints = list()

        self.current_stamp = rospy.Time.now()
        self.current_visual_tag_constraints = list()
        self.visual_tags_enabled = visual_tags_enabled

        self.continuous_estimation (SetBoolRequest(continuous_estimation))
        self.estimation_rate = 50 # Hz

        self.subscribers = ros_tools.createSubscribers (self, "/agimus", self.subscribersDict)
        self.publishers  = ros_tools.createPublishers ("/agimus", self.publishersDict)
        self.services    = ros_tools.createServices (self, "/agimus", self.servicesDict)
        self.joint_state_subs = rospy.Subscriber (joint_states_topic, JointState, self.get_joint_state)

    def continuous_estimation(self, msg):
        self.run_continuous_estimation = msg.data
	rospy.loginfo ("Run continuous estimation: {0}".format(self.run_continuous_estimation))
        return True, "ok"

    def spin (self):
        rate = rospy.Rate(self.estimation_rate)
        while not rospy.is_shutdown():
            if self.run_continuous_estimation and self.last_stamp_is_ready:
                rospy.logdebug("Runnning estimation...")
                self.estimation()
            else:
                rospy.logdebug ("run continuous estimation."
                        +"run_continuous_estimation={0}, last_stamp_is_ready={1}"
                        .format(self.run_continuous_estimation, self.last_stamp_is_ready))
            rate.sleep()

    def estimation (self, msg=None):
        self.mutex.acquire()

        try:
            hpp = self.hpp()
            q_current = hpp.robot.getCurrentConfig()

            self._initialize_constraints (q_current)

            # The optimization expects a configuration which already satisfies the constraints
            projOk, q_projected, error = hpp.problem.applyConstraints (q_current)

            if projOk:
                optOk, q_estimated, error = hpp.problem.optimize (q_projected)
                if not optOk:
                    from numpy.linalg import norm
                    errNorm = norm(error)
                    if errNorm > 1e-2:
                      rospy.logwarn_throttle (1 ,"Optimisation failed ? error norm: {0}".format(errNorm))
                      rospy.logdebug_throttle (1 ,"estimated == projected: {0}".format(q_projected==q_estimated))
                    else:
                      rospy.loginfo_throttle (1 ,"Optimisation failed ? error norm: {0}".format(errNorm))
                    rospy.logdebug_throttle (1 ,"Error {0}".format(error))

                valid, msg = hpp.robot.isConfigValid (q_estimated)
                if not valid:
                    rospy.logwarn_throttle (1, "Estimation in collision: {0}".format(msg))

                self.publishers["estimation"]["semantic"].publish (q_estimated)

                self.publish_state (hpp)
            else:
                hpp.robot.setCurrentConfig (q_current)
                q_estimated = q_current
                rospy.logwarn_throttle (1, "Could not apply the constraints {0}".format(error))
        except Exception as e:
            rospy.logerr_throttle (1, str(e))
            rospy.logerr_throttle (1, traceback.format_exc())
        finally:
            self.last_stamp_is_ready = False
            self.mutex.release()

    ## Publish tranforms to tf
    # By default, only the child joints of universe are published.
    def publish_state (self, hpp):
        robot_name = hpp.robot.getRobotName()
        if not hasattr(self, 'universe_child_joint_names'):
            self.universe_child_joint_names = [ jn for jn in hpp.robot.getJointNames() if "universe" == hpp.robot.getParentJointName(jn) ]
            rospy.loginfo("Will publish joints {0}".format(self.universe_child_joint_names))
        for jn in self.universe_child_joint_names:
            links = hpp.robot.getLinkNames(jn)
            for l in links:
                T = hpp.robot.getLinkPosition (l)
                if l.startswith(robot_name):
                    name = l[len(robot_name)+1:]
                else:
                    name = l
                self.tf_pub.sendTransform (T[0:3], T[3:7], self.last_stamp, name, self.tf_root)
        # Publish the robot link as estimated.
        robot_joints = filter(lambda x: x.startswith(robot_name), hpp.robot.getAllJointNames())
        for jn in robot_joints:
            links = hpp.robot.getLinkNames(jn)
            for name in links:
                T = hpp.robot.getLinkPosition (name)
                self.tf_pub.sendTransform (T[0:3], T[3:7], self.last_stamp, name, self.tf_root)

    def _initialize_constraints (self, q_current):
        from CORBA import UserException
        hpp = self.hpp()

        hpp.problem.resetConstraints()

        if hasattr(self, "manip"): # hpp-manipulation:
            # Guess current state
            # TODO Add a topic that provides to this node the expected current state (from planning)
            manip = self.manip ()
            try:
                state_id = manip.graph.getNode (q_current)
                rospy.loginfo_throttle(1, "At {0}, current state: {1}".format(self.last_stamp, state_id))
            except UserException:
                if hasattr(self, "last_state_id"): # hpp-manipulation:
                    state_id = self.last_state_id
                    rospy.logwarn_throttle(1, "At {0}, assumed last state: {1}".format(self.last_stamp, state_id))
                else:
                    state_id = rospy.get_param ("default_state_id")
                    rospy.logwarn_throttle(1, "At {0}, assumed default current state: {1}".format(self.last_stamp, state_id))
            self.last_state_id = state_id
            self.publishers["estimation"]["state_id"].publish (state_id)

            # copy constraint from state
            manip.problem.setConstraints (state_id, True)
            hpp.problem.addLockedJointConstraints("unused", self.locked_joints)
        else:
            # hpp-corbaserver: setNumericalConstraints
            default_constraints = rospy.get_param ("default_constraints")
            hpp.problem.addLockedJointConstraints("unused", self.locked_joints)
            hpp.problem.addNumericalConstraints ("constraints",
                    default_constraints,
                    [ 0 for _ in default_constraints ])

        # TODO we should solve the constraints, then add the cost and optimize.
        if len(self.last_visual_tag_constraints) > 0:
            rospy.loginfo_throttle(1, "Adding {0}".format(self.last_visual_tag_constraints))
            hpp.problem.addNumericalConstraints ("unused", self.last_visual_tag_constraints,
                    [ 1 for _ in self.last_visual_tag_constraints ])
            hpp.problem.setNumericalConstraintsLastPriorityOptional (True)

    def get_joint_state (self, js_msg):
        from CORBA import UserException
        self.mutex.acquire()
        try:
            hpp = self.hpp()
            robot_name = hpp.robot.getRobotName()
            if len(robot_name) > 0: robot_name = robot_name + "/"
            for jn, q in zip(js_msg.name, js_msg.position):
                name = robot_name + jn
                jt = hpp.robot.getJointType(name)
                if jt.startswith("JointModelRUB"):
                    assert hpp.robot.getJointConfigSize(name) == 2, name + " is not of size 2"
                    qjoint = [cos(q), sin(q)]
                else:
                    assert hpp.robot.getJointConfigSize(name) == 1, name + " is not of size 1"
                    # Check joint bounds
                    bounds = hpp.robot.getJointBounds(name)
                    if q-bounds[0] < -1e-3 or q-bounds[1] > 1e-3:
                        rospy.logwarn_throttle(1, "Current state {1} of joint {0} out of bounds {2}"
                            .format(name, q, bounds))
                    qjoint = [min(bounds[1],max(bounds[0],q)),]
                hpp.problem.createLockedJoint ('lock_' + name, name, qjoint)
            if len(self.locked_joints) == 0:
                self.locked_joints = tuple(['lock_'+robot_name+n for n in js_msg.name])
	except UserException as e:
            rospy.logerr ("Cannot get joint state: {0}".format(e))
        finally:
            self.mutex.release()

    def _get_transformation_constraint (self,
            joint1, joint2, transform,
            prefix = "", orientationWeight = 1.):
        hpp = self.hpp()

        # Create a relative transformation constraint
        j1 = joint1 if "/" in joint1 else self.robot_name + "/" + joint1
        j2 = joint2 if "/" in joint2 else self.robot_name + "/" + joint2
        name = prefix + j1 + "_" + j2
        T = [ transform.translation.x,
              transform.translation.y,
              transform.translation.z,
              transform.rotation.x,
              transform.rotation.y,
              transform.rotation.z,
              transform.rotation.w,]
        if orientationWeight == 1.:
            names = ["T_"+name, ]
            hpp.problem.createTransformationConstraint (names[0], j1, j2, T, [True,]*6)
        else:
            from hpp import Quaternion
            names = ["P_"+name, "sO_"+name]
            hpp.problem.createPositionConstraint (names[0], j1, j2, T[:3], [0,0,0], [True,]*3)
            hpp.problem.createOrientationConstraint ("O_"+name, j1, j2, Quaternion(T[3:]).inv().toTuple(), [True,]*3)
            hpp.problem.scCreateScalarMultiply (names[1], orientationWeight, "O_"+name)
        return names

    def get_visual_tag (self, ts_msg):
        stamp = ts_msg.header.stamp
        if stamp < self.current_stamp: return

        rot = ts_msg.transform.rotation
        # Compute scalar product between Z axis of camera and of tag.
        # TODO Add a weight between translation and orientation
        # It should depend on:
        # - the distance (the farthest, the hardest it is to get the orientation)
        distW = 1.
        # - the above scalar product (the closest to 0, the hardest it is to get the orientation)
        from hpp import Quaternion
        from numpy import array
        oriW = - Quaternion([rot.x, rot.y, rot.z, rot.w,]).transform(array([0,0,1]))[2]
        # - the tag size (for an orthogonal tag, an error theta in orientation should be considered
        #   equivalent to an position error of theta * tag_size)
        tagsize = 0.063 * 4 # tag size * 4
        s = tagsize * oriW * distW
        try:
            self.mutex.acquire()

            names = self._get_transformation_constraint (
                ts_msg.header.frame_id, ts_msg.child_frame_id, ts_msg.transform,
                prefix="", orientationWeight = s)
            # If this tag is in the next image:
            if self.current_stamp < stamp:
                # Assume no more visual tag will be received from image at time current_stamp.
                self.last_stamp = self.current_stamp
                self.last_visual_tag_constraints = self.current_visual_tag_constraints
                # Reset for next image.
                self.current_stamp = stamp
                self.current_visual_tag_constraints = list()
                self.last_stamp_is_ready = True
            self.current_visual_tag_constraints.extend(names)
        finally:
            self.mutex.release()

    def get_base_pose_estimation (self, ts_msg):
        stamp = ts_msg.header.stamp
        if stamp < self.current_stamp: return

        self.mutex.acquire()
        try:
            hpp = self.hpp()
            robot_name = hpp.robot.getRobotName()

            names = self._get_transformation_constraint (
                    "universe", robot_name + "/root_joint", ts_msg.transform,
                    prefix="base/", orientationWeight=1.)

            # TODO we should consider the stamp which is the closest to self.current_stamp
            if names[0] not in self.current_visual_tag_constraints:
                self.current_visual_tag_constraints.extend(names)

            if not self.visual_tags_enabled:
                # Assume no more visual tag will be received from image at time current_stamp.
                self.last_stamp = self.current_stamp
                self.last_visual_tag_constraints = self.current_visual_tag_constraints
                # Reset for next image.
                self.current_stamp = stamp
                self.current_visual_tag_constraints = list()
                self.last_stamp_is_ready = True
        finally:
            self.mutex.release()
