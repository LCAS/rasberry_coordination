#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import rospy
import tf
import geometry_msgs.msg
from bayes_people_tracker.msg import PeopleStamped


class TFPublishers:

    @staticmethod
    def get_tf_convertor(agent_id, agent_type):
        tf_source_topic = {'short_robot': '/%s/robot_pose'%agent_id,
                           'tall_robot': '/%s/robot_pose'%agent_id,
                           'human_sim':'/%s/pose_stamped'%agent_id,
                           'human':'gps_positions'}
        a = [k for k in tf_source_topic.keys() if agent_type.startswith(k)]

        tf_source_type = {'short_robot': TFPublishers.PoseTFConvertor,
                          'tall_robot': TFPublishers.PoseTFConvertor,
                          'human_sim': TFPublishers.PoseStampedTFConvertor,
                          'human': TFPublishers.GPSPositionsTFPublisher}
        b = [k for k in tf_source_type.keys() if agent_type.startswith(k)]

        return tf_source_type[b[0]](agent_id, tf_source_topic[a[0]])

    class PoseTFConvertor(object):
        def __init__(self, agent_id, source_topic, log=False):
	        self.log = log
	        self.agent_id = agent_id
	        if agent_id == "":
	            self.base_frame = "base_link"
	        else:
	            self.base_frame = self.agent_id + "/base_link"
	        self.tf_broadcaster = tf.TransformBroadcaster()

	        self.pose_sub = rospy.Subscriber(source_topic, geometry_msgs.msg.Pose, self.pose_cb)

        def pose_cb(self, msg):
	        if self.log:
	            rospy.loginfo(msg.pose)
	        self.tf_broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
	                                          (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
	                                          rospy.Time.now(),
	                                          self.base_frame,
	                                          "map")


    class PoseStampedTFConvertor(object):
        def __init__(self, agent_id, source_topic, log=False):
	        self.log = log
	        self.agent_id = agent_id
	        self.pose_sub = rospy.Subscriber(source_topic, geometry_msgs.msg.PoseStamped, self.pose_cb)
	        if agent_id == "":
	            self.base_frame = "base_link"
	        else:
	            self.base_frame = self.agent_id + "/base_link"
	        self.tf_broadcaster = tf.TransformBroadcaster()

        def pose_cb(self, msg):
	        if self.log:
	            rospy.loginfo(msg.pose)
	        self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
	                                          (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
	                                          rospy.Time.now(),
	                                          self.base_frame,
	                                          "map")


    class GPSPositionsTFPublisher(object):
        def __init__(self, agent_id, source_topic, log=False):
	        self.log = log
	        self.agent_id = agent_id
	        self.gps_sub = rospy.Subscriber(source_topic, PeopleStamped, self.gps_cb)
	        if agent_id == "":
	            self.base_frame = "base_link"
	        else:
	            self.base_frame = self.agent_id + "/base_link"
	        self.tf_broadcaster = tf.TransformBroadcaster()

        def gps_cb(self, msg):
	        person = [p.person for p in msg.people if p.person.name == self.agent_id and p.person.position != geometry_msgs.msg.Point()]
	        if len(person) < 1: return
	        person = person[0]

	        if self.log:
	            rospy.loginfo(msg.pose)
	        self.tf_broadcaster.sendTransform((p.person.position.x, p.person.position.y, p.person.position.z),
	                                          (0, 0, 0, 1),
	                                          rospy.Time.now(), self.base_frame,"map")




