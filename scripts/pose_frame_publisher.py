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
        self.pose_sub = rospy.Subscriber(self.robot_name+"/robot_pose", geometry_msgs.msg.Pose, self.pose_cb)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def pose_cb(self, msg):
        self.tf_broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                                          (msg.position.x, msg.position.y, msg.position.z, msg.position.w),
                                          rospy.Time.now(),
                                          self.robot_name + "/base_link",
                                          "map")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print (sys.argv)
        raise Exception("not enough arguments")
    else:
        robot_name = sys.argv[1]
        print (robot_name)

    pose_frame_publisher = PoseFramePublisher(robot_name)

    rospy.spin()