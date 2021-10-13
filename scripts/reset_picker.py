#!/usr/bin/env python

import rospy
from std_msgs.msg import String as StdString

if __name__ == "__main__":
	picker = "picker01"

	rospy.init_node('reset_picker')
	rospy.loginfo('Node Initialised')
	rospy.sleep(5)

	pub = rospy.Publisher('/car_client/set_states', StdString, queue_size=5, latch=True)
	rospy.loginfo('Publisher Initialised')

	msg = StdString('{\"user\":\"%s\", \"state\":\"%s\"}' % (picker, "INIT"))
	rospy.loginfo(msg)

	pub.publish(msg)
	rospy.loginfo('Message Published')

	rospy.loginfo('Entering SPIN')
	rospy.spin()
