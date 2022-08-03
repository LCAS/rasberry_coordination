#! /usr/bin/env python

from sys import argv
import rospy
from visualization_msgs.msg import MarkerArray


class RViZPublisher(object):
    def __init__(self, agent_id):
        self.sub = rospy.Subscriber("/%s/vis" % agent_id, MarkerArray, self.new_vis_cb)
        self.pub = rospy.Publisher("/vis", MarkerArray, queue_size=5)

    def new_vis_cb(self, msg):
        self.pub.publish(msg)


if __name__ == '__main__':
    agent_id = argv[1]
    rospy.init_node('re_vis', anonymous=True)

    re_vis = RViZPublisher(agent_id=agent_id)

    rospy.spin()
