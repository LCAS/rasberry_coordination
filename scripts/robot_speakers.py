#! /usr/bin/env python

import os
import rospy
from std_msgs.msg import String

class Speaker(object):
    def __init__(self):
        self.speaker_cb_1 = rospy.Subscriber('/gofar_002/ui/speaker', String, self.callback, callback_args='gofar')
        self.speaker_cb_2 = rospy.Subscriber('/hunter2_002/ui/speaker', String, self.callback, callback_args='hunter')
        self.speaker_cb_3 = rospy.Subscriber('/thorvald_014/ui/speaker', String, self.callback, callback_args='014')
        self.speaker_cb_4 = rospy.Subscriber('/thorvald_023/ui/speaker', String, self.callback, callback_args='023')
        self.speaker_cb_5 = rospy.Subscriber('/thorvald_024/ui/speaker', String, self.callback, callback_args='024')
        self.speaker_cb_6 = rospy.Subscriber('/thorvald_025/ui/speaker', String, self.callback, callback_args='025')
        self.speaker_cb_7 = rospy.Subscriber('/thorvald_030/ui/speaker', String, self.callback, callback_args='030')

    def callback(self, msg, callback_args):
        print(msg.data)
        os.system('espeak "%s...%s"'%(callback_args, msg.data))

if __name__ == '__main__':
    rospy.init_node('local_robot_speaker', anonymous=False)
    SP = Speaker()
    rospy.spin()
