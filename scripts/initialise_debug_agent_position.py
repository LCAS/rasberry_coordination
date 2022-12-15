#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import tf
from sys import argv
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

"""
Sets initial position of debug robot
+ offers capability to set new pose manually
"""

class PoseManager(object):
    def __init__(self, agent_id, initial_pose):
        self.pose = initial_pose

        sub = rospy.Subscriber("/%s/new_pose" % agent_id, Pose, self.new_pose_cb)
        pub = rospy.Publisher("/%s/robot_pose" % agent_id, Pose, latch=True, queue_size=5)

        print("Pose initialised to: %s\n\n"%str(initial_pose))

        while not rospy.is_shutdown():
            pub.publish(self.pose)
            rospy.sleep(1.2)  # timeout to republish marker obj (must be < marker.lifetime)

    def new_pose_cb(self, msg):
        print("New pose set for: %s\n\n"%str(msg))
        self.pose = msg


if __name__ == '__main__':
    agent_id = argv[1]
    rospy.init_node('set_pose', anonymous=True)

    x = float(argv[2])
    y = float(argv[3])
    a = float(argv[4])
    q = tf.transformations.quaternion_from_euler(0, 0, a)

    pose = Pose(position=Point(x=x,y=y), orientation=Quaternion(z=q[2],w=q[3]))
    pose_manager = PoseManager(agent_id, pose)
