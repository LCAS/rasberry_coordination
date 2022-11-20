#!/usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rospy, rospkg
import yaml

from std_msgs.msg import Empty
from visualization_msgs.msg import MarkerArray

from rasberry_coordination.msg import MarkerDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.rviz_markers.markers import AgentMarker


class MarkerPublisher(object):
    def __init__(self):
        self.agents = dict()

        folderpath = rospkg.RosPack().get_path('rasberry_coordination')+"/src/rasberry_coordination/rviz_markers/" 

        with open(folderpath+"colors.yaml",    'r') as f: self.color_dict = yaml.safe_load(f)
        with open(folderpath+"components.yaml",'r') as f: self.component_dict = yaml.safe_load(f)
        with open(folderpath+"structures.yaml",'r') as f: self.structures_dict = yaml.safe_load(f)

        self.agents_to_render, self.agents_to_pop = [], []
        self.publish_time = rospy.get_rostime()

        self.marker_set_sub = rospy.Subscriber('/rasberry_coordination/set_marker', MarkerDetails, self.set_marker_cb)
        self.get_marker_pub = rospy.Publisher('/rasberry_coordination/get_markers', Empty, queue_size=5)
        self.marker_pub_all = rospy.Publisher("/vis_all/all", MarkerArray, queue_size=10)

    def set_marker_cb(self, msg):
        if msg.agent_id not in self.agents:
            logmsg(category="rviz", id=msg.agent_id, msg="new %s: %s"%(msg.type, msg.optional_color))

            dicts = {'color': self.color_dict, 'components': self.component_dict, 'structures': self.structures_dict[msg.type]}
            self.agents[msg.agent_id] = AgentMarker(msg.agent_id, dicts, msg.type, msg.optional_color)
            self.agents_to_render.append(msg.agent_id)

        else:
            if msg.optional_color == 'remove':
                logmsg(category="rviz", id=msg.agent_id, msg="del %s: %s"%(msg.type, msg.optional_color))
                self.agents_to_pop.append(msg.agent_id)
          
            elif self.agents[msg.agent_id].agent_color != msg.optional_color:
                logmsg(category="rviz", id=msg.agent_id, msg="mod %s: %s"%(msg.type, msg.optional_color))
                self.agents[msg.agent_id].agent_color = msg.optional_color
                self.agents_to_render.append(msg.agent_id)
    

    def run(self):
        rospy.sleep(1)
        self.get_marker_pub.publish(Empty())
        while not rospy.is_shutdown():

            #if there are any pending updates
            if (self.agents_to_render or self.agents_to_pop) or (rospy.get_rostime() - self.publish_time > rospy.Duration(5)):

                #delete any agents to remove
                for a in self.agents_to_pop:
                    del self.agents[a]

                #generate new markers
                for a in self.agents_to_render:
                    self.agents[a].generate_marker_array()

                self.agents_to_render, self.agents_to_pop = [], []

                #construct a full array
                marker_array = MarkerArray()
                for a in self.agents.values():
                    marker_array.markers += a.marker_array.markers

                #set the timeouts
                self.publish_time = rospy.get_rostime()
                for m in marker_array.markers:
                    m.header.stamp = self.publish_time
                    m.lifetime = rospy.Duration(10)

                self.marker_pub_all.publish(marker_array)

            rospy.sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("visualise_agent_markers")
    MP = MarkerPublisher()

    from rasberry_coordination.coordinator_tools import RootInspector
    RootInspector(topic='~root_inspector', root=MP)

    MP.run()

