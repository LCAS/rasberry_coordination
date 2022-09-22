#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rospy
from std_msgs.msg import String
from time import sleep

class Broadcaster(object):
    def __init__(self):
        self.pubs = dict()
        self.msg = dict()
        for a in ['thorvald_023', 'thorvald_024', 'thorvald_025', 'thorvald_030']:
            self.pubs[a] = rospy.Publisher('/%s/ui/speaker'%a, String, queue_size=5)
        sleep(2)
        self.general_cb = rospy.Subscriber('/broadcast', String, self.callback, '')
        self.picker_94_cb = rospy.Subscriber('/STD_v2_246f284a6c94/closest_node', String, self.callback, 'picker 9 4. ')
        self.picker_68_cb = rospy.Subscriber('/STD_v2_bcddc2cfcb68/closest_node', String, self.callback, 'picker 6 8. ')
        self.picker_f8_cb = rospy.Subscriber('/STD_v2_30c6f7e65af8/closest_node', String, self.callback, 'picker f 8. ')

    def callback(self, msg, publisher=''):
        msg.data = publisher + " " + msg.data
        if publisher not in self.msg:
            self.msg[publisher] = ""
        if self.msg[publisher] != msg.data:
            self.msg[publisher] = msg.data
            print(msg)
            for p in self.pubs.values():
                p.publish(msg)


if __name__ == '__main__':
    rospy.init_node('broadcaster', anonymous=False)
    BC = Broadcaster()
    rospy.spin()
