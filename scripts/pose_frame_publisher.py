#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""

import rospy
import tf
import sys
import geometry_msgs.msg


class PoseFramePublisher(object):
    """
    """
    def __init__(self, robot_name):
        """
        """
        self.robot_name = robot_name
        if robot_name == "":
            self.pose_sub = rospy.Subscriber("/robot_pose", geometry_msgs.msg.Pose, self.pose_cb)
            self.base_frame = "/base_link"
        else:
            self.pose_sub = rospy.Subscriber("/"+self.robot_name + "/robot_pose", geometry_msgs.msg.Pose, self.pose_cb)
            self.base_frame = self.robot_name + "/base_link"
        self.tf_broadcaster = tf.TransformBroadcaster()

    def pose_cb(self, msg):
        self.tf_broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                                          (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                                          rospy.Time.now(),
                                          self.base_link,
                                          "map")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        robot_name = ""
    else:
        robot_name = sys.argv[1]

    print (robot_name)

    rospy.init_node("pose_frame_publisher")

    pose_frame_publisher = PoseFramePublisher(robot_name)

    rospy.spin()