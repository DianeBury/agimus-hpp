import rospy

## Wait indefinitely for a service, return a ServiceProxy if found and a type has been provided
## \param srv the service name
## \param service_type of the service
## \param time after which a warning is printed using rospy.logwarn
## \return a proxy to the service if service_type is given
def wait_for_service(srv, service_type = None, time=0.2):
    try:
        rospy.wait_for_service(srv, time)
    except rospy.ROSException:
        rospy.logwarn("Waiting for service: {0}".format(srv))
        rospy.wait_for_service(srv)
        rospy.logwarn("Service {0} found.".format(srv))
    if service_type is not None:
        return rospy.ServiceProxy(srv, service_type)
    else:
        return None

## Internal function. Use createSubscribers or createPublishers instead.
## \param subscribe boolean whether this node should subscribe to the topics.
##        If False, this node publishes to the topics.
def _createTopics (object, namespace, topics, subscribe):
    if isinstance(topics, dict):
        rets = dict ()
        for k, v in topics.items():
            rets[k] = _createTopics(object, namespace + "/" + k, v, subscribe)
        return rets
    else:
        if subscribe:
            try:
                callback = getattr(object, topics[1])
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(object.__class__.__name__, topics[1]))
            return rospy.Subscriber(namespace, topics[0], callback)
        else:
            return rospy.Publisher(namespace, topics[0], queue_size = topics[1])

## Create rospy.Subscriber.
## \param object the object containing the callbacks.
## \param namespace prefix for the topic names
## \param topics a dictionary whose keys are topic names and values are a list <tt>[ Type, name_of_callback_in_object ]</tt>.
## \return a hierarchy of dictionary having the same layout as \c topics and whose leaves are rospy.Subscriber object.
##
## For instance:
## \code
## topics = { "foo" : { "topic1": [ std_msgs.msg.Empty, "function" ] }, }
## subscribers = createSubscribers (obj, "/bar", topics)
## # subscribers = { "foo" : {
## #                   "topic1": rospy.Subscriber ("/bar/foo/topic1", std_msgs.msg.Empty, obj.function)
## #                   }, }
## \endcode
def createSubscribers (object, namespace, topics):
    return _createTopics (object, namespace, topics, True)

## Create rospy.Publisher.
##
## \param namespace prefix for the topics names
## \param topics a dictionary whose keys are topic names and values are a list <tt>[ Type, queue_size ]</tt>.
## \return a hierarchy of dictionary having the same layout as \c topics and whose leaves are rospy.Publisher object.
##
## \sa createSubscribers
def createPublishers (namespace, topics):
    return _createTopics (None, namespace, topics, False)

def _createServices (object, namespace, services, serve):
    """
    \param serve boolean whether this node should serve or use the topics.
    """
    if isinstance(services, dict):
        rets = dict ()
        for k, v in services.items():
            rets[k] = _createServices(object, namespace + "/" + k, v, serve)
        return rets
    else:
        if serve:
            try:
                callback = getattr(object, services[1])
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(object.__class__.__name__, services[1]))
            return rospy.Service(namespace, services[0], callback)
        else:
            return wait_for_service(namespace, services[0])

## Create rospy.Service.
##
## \param object the object containing the callbacks.
## \param namespace prefix for the services names
## \param services a dictionary whose keys are topic names and values are a list <tt>[ Type, name_of_callback_in_object ]</tt>.
## \return a hierarchy of dictionary having the same layout as \c topics and whose leaves are rospy.Service object.
## \sa createSubscribers
def createServices (object, namespace, services):
    return _createServices (object, namespace, services, True)

## Create rospy.ServiceProxy.
##
## \param namespace prefix for the services names
## \param services a dictionary whose keys are topic names and values are a list <tt>[ Type, ]</tt>.
## \return a hierarchy of dictionary having the same layout as \c topics and whose leaves are rospy.ServiceProxy object.
## \sa createSubscribers
def createServiceProxies (namespace, services):
    return _createServices (None, namespace, services, False)
