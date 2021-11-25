#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import rospy
import tf
import geometry_msgs.msg


class PoseBaseFramePublisher(object):
    """
    """
    def __init__(self, agent_name, pose_topic, log=False):
        """
        """
        self.log = log
        self.agent_name = agent_name
        if agent_name == "":
            self.base_frame = "base_link"
        else:
            self.base_frame = self.agent_name + "/base_link"
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        self.pose_sub = rospy.Subscriber(pose_topic, geometry_msgs.msg.Pose, self.pose_cb)

    def pose_cb(self, msg):
        if self.log:
            rospy.loginfo(msg.pose)
        self.tf_broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                                          (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                                          rospy.Time.now(),
                                          self.base_frame,
                                          "map")


class PoseStampedBaseFramePublisher(object):
    """
    """
    def __init__(self, agent_name, posestamped_topic, log=False):
        """
        """
        self.log = log
        self.agent_name = agent_name
        self.pose_sub = rospy.Subscriber(posestamped_topic, geometry_msgs.msg.PoseStamped, self.pose_cb)
        if agent_name == "":
            self.base_frame = "base_link"
        else:
            self.base_frame = self.agent_name + "/base_link"
        self.tf_broadcaster = tf.TransformBroadcaster()

    def pose_cb(self, msg):
        if self.log:
            rospy.loginfo(msg.pose)
        self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                                          (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
                                          rospy.Time.now(),
                                          self.base_frame,
                                          "map")