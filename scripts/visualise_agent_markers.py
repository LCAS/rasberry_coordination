#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_coordination.base_frame_publisher
import rasberry_coordination.markers as markers
import rasberry_coordination.msg
import sys
import rasberry_des.config_utils
import geometry_msgs.msg
from std_msgs.msg import String


class MarkerPublisher:

    def __init__(self, config):
        self.config = config

        # Identify agents
        if config['version'] == '1.1.0':
            r = [self.config_get('robot_id', config=robot) for robot in self.config_get('spawn_list')[1:]]
            p = [] + self.config_get("picker_ids")
            vp = [] + self.config_get("virtual_picker_ids")
        else:
            r = [agent['agent_id'] for agent in config['agent_list'] if agent['setup']['manager'] == "RobotManager"]
            p = [agent['agent_id'] for agent in config['agent_list'] if agent['setup']['manager'] == "PickerManager" and agent['setup']['physical'] == "True"]
            vp = [agent['agent_id'] for agent in config['agent_list'] if agent['setup']['manager'] == "PickerManager" and agent['setup']['physical'] == "False"]
        self.robot_ids = r
        self.picker_ids = p
        self.virtual_picker_ids = vp

        # Initialise publisher handlers
        self.thorvald_marker_publishers = {}
        self.picker_marker_publishers = {}
        self.virtual_picker_marker_publishers = {}
        self.base_frame_publishers = {}

        # Add markers defined in config file
        self.startup_markers()

        # Listen for additional markers to be added
        self.marker_add_sub = rospy.Subscriber('/rasberry_coordination/marker_add',
                                               rasberry_coordination.msg.MarkerDetails,
                                               self.add_marker_cb)
        self.marker_remove_sub = rospy.Subscriber('/rasberry_coordination/marker_remove',
                                                  rasberry_coordination.msg.MarkerDetails,
                                                  self.remove_marker_cb)

    # Return item from config if exists
    def config_get(self, item, config=None):
        if config is None:
            config = self.config
        if item in config:
            return config[item]
        else:
            return []

    # Add the markers which were defined in the map_config file
    def startup_markers(self):
        for robot_id in self.robot_ids:
            print(self.robot_ids)
            print("Adding robot " + robot_id)
            self.add_robot(robot_id)
        for picker_id in self.picker_ids:
            print("Adding picker " + picker_id)
            self.add_picker(picker_id)
        for virtual_picker_id in self.virtual_picker_ids:
            print("Adding virtual picker " + virtual_picker_id)
            self.add_virtual_picker(virtual_picker_id)

    # Marker add callback
    def add_marker_cb(self, msg):
        print("Adding " + msg.type + " " + msg.name)
        if msg.type == "robot":
            self.robot_ids.append(msg.name)
            self.add_robot(msg.name, msg.optional_color)
            print(self.thorvald_marker_publishers.keys())
        elif msg.type == "picker":
            self.picker_ids.append(msg.name)
            self.add_picker(msg.name)
            print(self.picker_marker_publishers.keys())
        elif msg.type == "virtual_picker":
            self.virtual_picker_ids.append(msg.name)
            self.add_virtual_picker(msg.name)
            print(self.virtual_picker_marker_publishers.keys())
        return 0

    # Setup robot marker
    def add_robot(self, id, color=''):
        if id in self.thorvald_marker_publishers:
            self.thorvald_marker_publishers.pop(id)
            self.base_frame_publishers.pop(id)

        self.thorvald_marker_publishers[id] = markers.ColoredThorvaldMarkerPublisher(id, color)
        topic = "/" + id + "/robot_pose"
        pub = rasberry_coordination.base_frame_publisher.PoseBaseFramePublisher(id, topic)
        self.base_frame_publishers[id] = pub

    # Setup picker marker
    def add_picker(self, id):
        self.picker_marker_publishers[id] = markers.HumanMarkerPublisher(id)
        topic = "/" + id + "/posestamped"
        pub = rasberry_coordination.base_frame_publisher.PoseStampedBaseFramePublisher(id, topic)
        self.base_frame_publishers[id] = pub

    # Setup virtual picker marker
    def add_virtual_picker(self, id):
        self.virtual_picker_marker_publishers[id] = markers.HumanMarkerPublisher(id)
        topic = "/" + id + "/posestamped"
        pub = rasberry_coordination.base_frame_publisher.PoseStampedBaseFramePublisher(id, topic)
        self.base_frame_publishers[id] = pub

    # Marker remove callback
    def remove_marker_cb(self, msg):
        print("Removing " + msg.type + " " + msg.name)
        if msg.type == "robot":
            self.thorvald_marker_publishers.pop(msg.name, None)
            self.robot_ids.remove(msg.name)
            print(self.thorvald_marker_publishers.keys())
        elif msg.type == "picker":
            self.picker_marker_publishers.pop(msg.name, None)
            self.picker_ids.remove(msg.name)
            print(self.picker_marker_publishers.keys())
        elif msg.type == "virtual_picker":
            self.virtual_picker_marker_publishers.pop(msg.name, None)
            self.virtual_picker_ids.remove(msg.name)
            print(self.virtual_picker_marker_publishers.keys())
        self.base_frame_publishers.pop(msg.name, None)
        return 0

    # Spawn markers in rviz
    def run(self):
        while not rospy.is_shutdown():
            [pub.publish() for pub in self.thorvald_marker_publishers.values()]
            [pub.publish() for pub in self.picker_marker_publishers.values()]
            [pub.publish() for pub in self.virtual_picker_marker_publishers.values()]
            rospy.sleep(1.0)


if __name__ == "__main__":

    # Ensure config file path passed to script
    if len(sys.argv) < 2:
        raise Exception("usage: visualise_agent_markers.py <config_file>")
    else:
        config_file = sys.argv[1]

    # Read config data
    config_data = rasberry_des.config_utils.get_config_data(config_file)
    config_keys = rasberry_des.config_utils.get_config_keys(config_file)

    # Check for required parameters
    req_params = ["agent_list"]
    for key in config_keys:
        if key in req_params:
            req_params.remove(key)

    if len(req_params) != 0:
        raise Exception("not all required keys are set in the config file")
    elif config_data["agent_list"].__class__ != list:
        raise Exception("robot_ids should be a list in the config file")
    elif len(config_data["agent_list"]) == 0:
        raise Exception("robot_ids should not be an empty list in the config file")

    # Start node
    rospy.init_node("visualise_agent_markers")

    # Run marker publisher
    MP = MarkerPublisher(config_data)
    MP.run()
