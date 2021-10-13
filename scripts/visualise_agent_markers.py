#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
from rasberry_coordination.base_frame_publisher import PoseBaseFramePublisher
import rasberry_coordination.markers as markers
import rasberry_coordination.msg
import sys
import rasberry_des.config_utils
import geometry_msgs.msg
from std_msgs.msg import String
from rasberry_coordination.msg import MarkerDetails

class AgentMarker:
    def __init__(self, msg):
        self.type = msg.type
        self.agent_id = msg.agent_id
        self.color = msg.optional_color

        models = {'short_robot': markers.ColoredThorvaldMarkerPublisher,
                  'tall_robot':  markers.ColoredTallThorvaldMarkerPublisher,
                  'human':       markers.HumanMarkerPublisher}
        self.marker = models[self.type](self.agent_id, self.color)

        topics = {"short_robot": "/%s/robot_pose",
                  "tall_robot":  "/%s/robot_pose",
                  "human":       "/%s/pose_stamped"}
        self.base_frame_publisher = PoseBaseFramePublisher(id, topics[self.type]%self.agent_id)

    def publish_marker(self): self.marker.publish()

class MarkerPublisher:

    def __init__(self):
        self.agents = dict()

        # Listen for markers to be added
        self.marker_set_sub = rospy.Subscriber('/rasberry_coordination/set_marker', MarkerDetails, self.set_marker_cb)

    """ Main Callback """
    def set_marker_cb(self, msg):
        logmsg(category="rviz", id=msg.agent_id, msg="Setting %s %s(%s)"%(msg.type,msg.agent_id,msg.optional_color))

        if msg.optional_color == 'remove':
            # Remove Agent from RViZ
            self.agents.pop(msg.agent_id)

        else:
            # Add Agent to RViZ
            self.agents[msg.agent_id] = AgentMarker(msg)

    # Spawn markers in RViZ
    def run(self):
        while not rospy.is_shutdown():
            for agent in self.agents:
                agent.publish_marker()
                rospy.sleep(1.0)


if __name__ == "__main__":

    # Start node
    rospy.init_node("visualise_agent_markers")

    # Run marker publisher
    MP = MarkerPublisher()
    MP.run()
