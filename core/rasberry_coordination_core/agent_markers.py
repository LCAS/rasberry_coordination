#!/usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date: 10/aug/2022
# ----------------------------------

import os
import yaml

import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from rasberry_coordination_msgs.msg import MarkerDetails

from rasberry_coordination_core.rviz_markers.markers import AgentMarker


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('rviz_markers')

        self.agents = dict()

        #TODO: replace with args
        components_path = os.getenv('RVIZ_COMPONENTS_CONFIG', None)
        structures_path = os.getenv('RVIZ_STRUCTURES_CONFIG', None)
        print(structures_path)

        with open(components_path,'r') as f: self.component_dict = yaml.safe_load(f)
        with open(structures_path,'r') as f: self.structures_dict = yaml.safe_load(f)
        self.agents_to_render, self.agents_to_pop = [], []
        self.publish_time = time.time()

        self.marker_set_sub = self.create_subscription(MarkerDetails, "/rasberry_coordination/set_marker", self.set_marker_cb, 10)
        self.get_marker_pub = self.create_publisher(Empty, "/rasberry_coordination/get_markers", 5)
        self.marker_pub_all = self.create_publisher(MarkerArray, "/vis_all/all", 10)

    def set_marker_cb(self, msg):
    
        # If agent does not exist, create new container with marker directories
        if msg.id not in self.agents:
            print("%s new %s: %s | %s (%s)"%(msg.id, msg.structure, str(msg.colour).replace('\n',''), msg.tf_source_topic, msg.tf_source_type))
            dicts = {'components': self.component_dict, 'structures': self.structures_dict}
            self.agents[msg.id] = AgentMarker(msg, dicts, self)
            self.agents_to_render.append(msg.id)
            return

        # If no structure given, delete agent
        if msg.structure == '':
            print("%s del"%(msg.agent_id))
            self.agents_to_pop.append(msg.agent_id)
            return

        # If given description is unchanged, skip
        a = self.agents[msg.id]
        if a.structure == msg.structure and a.colour == msg.colour and not msg.pose:
            return

        # Save structure change and colour change
        if a.structure != msg.structure:
            print("%s mod struct: %s -> %s"%(msg.id, a.structure, msg.structure))
            a.structure = msg.structure
        if a.colour != [msg.colour.r, msg.colour.g, msg.colour.b, msg.colour.a]:
            print("%s mod colour: %s -> %s"%(msg.id, a.colour, str(msg.colour).replace('\n','')))
            a.colour = [msg.colour.r, msg.colour.g, msg.colour.b, msg.colour.a]
        self.agents_to_render.append(msg.id)

        # Save new pose for tf system to use
        if msg.pose and msg.pose.position != Point():
            print("%s mod pose: %s"%(msg.id, str(msg.pose.position).replace('\n','')))
            pos, ori = msg.pose.position, msg.pose.orientation
            a.tf.pose = [[pos.x, pos.y, pos.z],[ori.x, ori.y, ori.z, ori.w]]


    def run(self):
        time.sleep(1)
        self.get_marker_pub.publish(Empty())
        i = 0
        while rclpy.ok():

            #if there are any pending updates
            if (self.agents_to_render or self.agents_to_pop) or (time.time() - self.publish_time > 5):
                print("Cycle: %s"%i)
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
                self.publish_time = time.time()
                for m in marker_array.markers:
                    m.header.stamp = self.publish_time
                    m.lifetime = 10

                self.marker_pub_all.publish(marker_array)

            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)

    MP = MarkerPublisher()
    MP.run()

    MP.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

