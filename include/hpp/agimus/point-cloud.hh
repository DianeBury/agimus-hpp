// Copyright 2021, 2022, CNRS, Airbus SAS

// Author: Florent Lamiraux

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef HPP_AGIMUS_POINT_CLOUD_HH
#define HPP_AGIMUS_POINT_CLOUD_HH

#include <boost/thread/mutex.hpp>

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

#include <hpp/util/pointer.hh>
#include <hpp/agimus/fwd.hh>

namespace hpp {
  namespace agimus {
    class PointCloud
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef hpp::fcl::OcTreePtr_t OcTreePtr_t;
      static PointCloudPtr_t create (const ProblemSolverPtr_t& ps)
      {
	PointCloudPtr_t ptr (new PointCloud(ps));
	ptr->init(ptr);
	return ptr;
      }
      /// Initialize a ROS node with given name
      bool initializeRosNode (const std::string& name, bool anonymous);

      void shutdownRos ();
      /// Build an octree from a point cloud read on a ROS topic
      /// \param octreeFrame frame to which the octree is attached.
      /// \param topic name of the topic. Topic should be of type
      ///        sensor_msgs/PointCloud2
      /// \param sensorFrame name of the frame in which the point cloud is
      ///        expressed.
      /// \param resolution resolution of the Octree built from the point cloud.
      /// \param configuration configuration of the system. Used to compute the
      ///        pose of the joint holding the octree with respect to the
      ///        sensor frame.
      /// \param timeOut time after which the function returns error if no data
      ///        has been published (in seconds).
      bool buildPointCloud(const std::string& octreeFrame,
			 const std::string& topic,
			 const std::string& sensorFrame,
			 value_type resolution, const vector_t& configuration,
			 value_type timeOut,
			 bool newPointCloud);
      /// Remove the octree attached to octreeFrame
      /// This also removes the octree in the GUI
      void removeOctree(const std::string& octreeFrame);
      /// Set bounds on distance of points to sensor
      /// Points at a distance outside this interval are ignored.
      void setDistanceBounds(value_type min, value_type max);
      /// Set whether to display octree in gepetto-viewer
      void setDisplay(bool flag);
      /// Callback to the point cloud topic
      void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& data);
      /// Set three points belonging to the object plan, in the object frame
      /// The (oriented) normal will be computed as $AC \times AB$ and
      /// all points behind the plan will be filtered out.
      /// The margin is set to 0.
      /// \param pointA point in the object plan, in the reference frame
      /// \param pointB point in the object plan, in the reference frame
      /// \param pointC point in the object plan, in the reference frame
      void setObjectPlan(const vector3_t& pointA,
                         const vector3_t& pointB,
                         const vector3_t& pointC)
      {
        plaquePoint_ = pointA;
        plaqueNormalVector_ = (pointC - pointA).cross(pointB - pointA);
        plaqueNormalVector_ = plaqueNormalVector_ / plaqueNormalVector_.norm();
        filterBehindPlan_ = true;
        objectPlanMargin_ = 0;
      }
      /// Stop filtering the points behind the object plan
      void removeObjectPlan()
      {
        filterBehindPlan_ = false;
      }
      /// Set the margin behind which the points of the
      /// point cloud get filtered out
      /// \param margin points at less than the margin distance
      ///        of the plan are filtered out
      void setObjectPlanMargin(value_type margin)
      {
        objectPlanMargin_ = margin;
      }

      /// Shut down ROS
      ~PointCloud();
    private:
      /// Constructor
      PointCloud(const ProblemSolverPtr_t& ps);
      void init (const PointCloudWkPtr_t)
      {}
      /// Express point cloud in octreeFrame
      void movePointCloud(const std::string& octreeFrame,
			  const std::string& sensorFrame,
			  const vector_t& configuration);
      /// Test if a point is in the wanted range
      bool filterPoint(uint32_t pointcloud_id, uint32_t row_id);

      void attachOctreeToRobot
      (const OcTreePtr_t& octree, const std::string& octreeFrame);
      bool displayOctree(const OcTreePtr_t& octree,
			 const std::string& octreeFrame);
      /// Remove the octree in the GUI
      bool hideOctree(const std::string& octreeFrame);
      ProblemSolverPtr_t problemSolver_;
      bool waitingForData_;
      boost::mutex mutex_;
      ros::NodeHandle* handle_;

      // Vector of the different point clouds measured
      std::vector<PointMatrix_t> pointsInSensorFrame_;
      PointMatrix_t pointsInLinkFrame_;
      value_type minDistance_, maxDistance_;
      bool display_;
      bool filterBehindPlan_;
      value_type objectPlanMargin_;
      // Point in the object plan, expressed in the object frame
      vector3_t plaquePoint_;
      // Normal to the object plan in the object frame
      vector3_t plaqueNormalVector_;
      std::string octreeFrame_;
      std::string sensorFrame_;
      bool newPointCloud_;

    }; // class PointCloud
  } // namespace agimus
} // namespace hpp

#endif // HPP_AGIMUS_POINT_CLOUD_HH
