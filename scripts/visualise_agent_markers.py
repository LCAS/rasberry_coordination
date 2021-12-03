#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
from rasberry_coordination.base_frame_publisher import PoseBaseFramePublisher, PoseStampedBaseFramePublisher
import rasberry_coordination.markers as markers
import rasberry_coordination.msg
import sys
import rasberry_des.config_utils
import geometry_msgs.msg
from std_msgs.msg import String, Empty
from rasberry_coordination.msg import MarkerDetails
from rasberry_coordination.coordinator_tools import logmsg


class AgentMarker(object):
    def __init__(self, msg):
        self.type = msg.type
        self.agent_id = msg.agent_id
        self.color = msg.optional_color

        models = {'short_robot': markers.ColoredThorvaldMarkerPublisher,
                  'tall_robot':  markers.ColoredTallThorvaldMarkerPublisher,
                  'human':       markers.HumanMarkerPublisher}
        self.marker = models[self.type](self.agent_id, self.color)

        topics = {"short_robot": ["/%s/robot_pose",   PoseBaseFramePublisher],
                  "tall_robot":  ["/%s/robot_pose",   PoseBaseFramePublisher],
                  "human":       ["/%s/posestamped", PoseStampedBaseFramePublisher]}
        self.base_frame_publisher = topics[self.type][1](self.agent_id, topics[self.type][0]%self.agent_id)

    def publish_marker(self): self.marker.publish()


class MarkerPublisher(object):
    def __init__(self):
        self.agents = dict()
        self.marker_set_sub = rospy.Subscriber('/rasberry_coordination/set_marker', MarkerDetails, self.set_marker_cb)
        self.get_marker_pub = rospy.Publisher('/rasberry_coordination/get_markers', Empty, queue_size=5)

    def set_marker_cb(self, msg):
        logmsg(category="rviz", id=msg.agent_id, msg="Setting %s %s(%s)"%(msg.type, msg.agent_id, msg.optional_color))
        if msg.optional_color == 'remove':
            # Remove Agent from RViZ
            self.agents.pop(msg.agent_id)
        else:
            # Add Agent to RViZ if same color agent is not already present
            if (msg.agent_id not in self.agents) or (self.agents[msg.agent_id].color != msg.optional_color):
                self.agents[msg.agent_id] = AgentMarker(msg)

    # Spawn markers in RViZ
    def run(self):
        rospy.sleep(5)
        self.get_marker_pub.publish(Empty())
        while not rospy.is_shutdown():
            for agent in self.agents.values():
                agent.publish_marker()
            rospy.sleep(0.5)


if __name__ == "__main__":
    # Start node
    rospy.init_node("visualise_agent_markers")

    # Create marker publisher
    MP = MarkerPublisher()

    # Launch inspector tool
    from rasberry_coordination.coordinator_tools import RootInspector
    RootInspector(topic='~root_inspector', root=MP)

    # Run marker publisher
    MP.run()





