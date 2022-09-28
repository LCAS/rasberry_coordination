#! /usr/bin/env python

import rospy
from topological_navigation_msgs.msg import GotoNodeActionGoal, GotoNodeActionResult
from time import sleep
from std_msgs.msg import String
import random

class ManualRoute:
    def __init__(self):
        self.sub = rospy.Subscriber('/topological_navigation/result', GotoNodeActionResult, self.callback)
        self.pub = rospy.Publisher('/topological_navigation/goal', GotoNodeActionGoal, queue_size=5)
        self.speaker = rospy.Publisher('/ui/speaker', String, queue_size=5)

        row_ids = [0.7, 1.5, 2.5, 3.5, 4.5, 5.3, 5.7, 6.5, 7.5, 8.5, 9.5, 10.3]
        #random.shuffle(row_ids)
        rows = ["r%s-c"%r for r in row_ids]

        route = []
        for i,r in enumerate(rows):
            ends = ["a","z"]
            #if i%2:
            #    ends = ["z","a"]
            route.append(r+ends[0])
            route.append(r+ends[1])
        print(route)
        print("\n"*3)
        sleep(5)
        self.route = route
        self.i = 0
        self.e = GotoNodeActionGoal()
        self.publish_next_node()

    def publish_next_node(self):
        e = GotoNodeActionGoal()
        e.header.seq = self.i*10
        e.header.stamp = rospy.Time.now()
        e.goal_id.stamp = rospy.Time.now()
        e.goal_id.id = "/route-2-"+str(e.goal_id.stamp.secs)+".0"+str(e.goal_id.stamp.nsecs)[:2]
        e.goal.target = self.route[self.i]
        e.goal.no_orientation = False
        print("\n\n\n-----------------------")
        print(e)
        self.e = e
        self.pub.publish(self.e)
        self.speaker.publish(String('new target: %s'%self.route[self.i].replace('-c','-c,'))
        self.i += 1

        """
        header:
          seq: 2
          stamp:
            secs: 1664199717
            nsecs:  32447099
          frame_id: ''
        goal_id:
          stamp:
            secs: 1664199717
            nsecs:  32426118
          id: "/visualise_map-2-1664199717.032"
        goal:
          target: "WayPoint71"
          no_orientation: False
        """

    def callback(self, msg):
        print("\n"*2)
        print(msg)
        if msg.result.success:
            self.publish_next_node()
        else:
            self.speaker.publish(String('again: %s'%self.route[self.i].replace('-c','-c,'))
            self.pub.publish(self.e)

if __name__ == '__main__':
    rospy.init_node('manual_route', anonymous=False)

    mr = ManualRoute()

    rospy.spin()
