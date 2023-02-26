#!/usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rospy, rospkg
import yaml
import os

from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray

from rasberry_coordination.msg import MarkerDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.rviz_markers.markers import AgentMarker


class MarkerPublisher(object):
    def __init__(self):
        self.agents = dict()

        internal_path = rospkg.RosPack().get_path('rasberry_coordination')+"/src/rasberry_coordination/rviz_markers/"
        structures_path = os.getenv('RVIZ_STRUCTURES_CONFIG', None)
        print(structures_path)

        with open(internal_path+"components.yaml",'r') as f: self.component_dict = yaml.safe_load(f)
        with open(structures_path,'r') as f: self.structures_dict = yaml.safe_load(f)
        self.agents_to_render, self.agents_to_pop = [], []
        self.publish_time = rospy.get_rostime()

        self.marker_set_sub = rospy.Subscriber('/rasberry_coordination/set_marker', MarkerDetails, self.set_marker_cb)
        self.get_marker_pub = rospy.Publisher('/rasberry_coordination/get_markers', Empty, queue_size=5)
        self.marker_pub_all = rospy.Publisher("/vis_all/all", MarkerArray, queue_size=10)

    def set_marker_cb(self, msg):

        # If agent does not exist, create new container with marker directories
        if msg.id not in self.agents:
            logmsg(category="rviz", id=msg.id, msg="new %s: %s | %s (%s)"%(msg.structure, str(msg.colour).replace('\n',''), msg.tf_source_topic, msg.tf_source_type))
            dicts = {'components': self.component_dict, 'structures': self.structures_dict}
            self.agents[msg.id] = AgentMarker(msg, dicts)
            self.agents_to_render.append(msg.id)
            return

        # If no structure given, delete agent
        if msg.structure == '':
            logmsg(category="rviz", id=msg.agent_id, msg="del: %s"%(msg.agent_id))
            self.agents_to_pop.append(msg.agent_id)
            return

        # If given description is unchanged, skip
        a = self.agents[msg.id]
        if a.structure == msg.structure and a.colour == msg.colour and not msg.pose:
            return

        # Save structure change and colour change
        if a.structure != msg.structure:
            logmsg(category="rviz", id=msg.id, msg="mod struct: %s -> %s"%(a.structure, msg.structure))
            a.structure = msg.structure
        if a.colour != [msg.colour.r, msg.colour.g, msg.colour.b, msg.colour.a]:
            logmsg(category="rviz", id=msg.id, msg="mod colour: %s -> %s"%(a.colour, str(msg.colour).replace('\n','')))
            a.colour = [msg.colour.r, msg.colour.g, msg.colour.b, msg.colour.a]
        self.agents_to_render.append(msg.id)

        # Save new pose for tf system to use
        if msg.pose and msg.pose.position != Point():
            logmsg(category="rviz", id=msg.id, msg="mod pose: %s"%(str(msg.pose.position).replace('\n','')))
            pos, ori = msg.pose.position, msg.pose.orientation
            a.tf.pose = [[pos.x, pos.y, pos.z],[ori.x, ori.y, ori.z, ori.w]]


    def run(self):
        rospy.sleep(1)
        self.get_marker_pub.publish(Empty())
        i = 0
        while not rospy.is_shutdown():

            #if there are any pending updates
            if (self.agents_to_render or self.agents_to_pop) or (rospy.get_rostime() - self.publish_time > rospy.Duration(5)):
                logmsg(category="null")
                logmsg(category="rviz", msg="Cycle: %s"%i)
                i+=1

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
                    a.tf.cycle_tf()
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
