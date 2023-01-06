#!/usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date: 10/aug/2022
# ----------------------------------


from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker

from tf.transformations import quaternion_from_euler

from rasberry_coordination.rviz_markers.tf_publisher import TFPublishers
from rasberry_coordination.coordinator_tools import logmsg


class AgentMarker(object):
    def __init__(self, agent_id, dicts, agent_type, agent_color, topic, location):
        self.color_dict = dicts['color']
        self.structure_dict = dicts['structures']
        self.components_dict = dicts['components']
        self.agent_id = agent_id
        self.type = agent_type
        self.agent_color = agent_color
        self.marker_array = MarkerArray()
        source = location if agent_type == 'special_node' else topic
        self.tf = TFPublishers.get_tf_convertor(agent_id, agent_type, source)

    def generate_marker_array(self):
        logmsg(category="rviz", id=self.agent_id, msg="generating marker")
        marker_array = MarkerArray()
        if self.type not in self.structure_dict:
            self.type = 'short_robot_load_4'
        for component_type,items in self.structure_dict[self.type].items():
            for i in items:
                c = self.get_component(component_type, ns=i[0], marker_index=i[1], position=i[2])
                marker_array.markers.append(c)
        self.marker_array = marker_array

    def get_component(self, component_type, ns, marker_index, position):
        component_dict = self.components_dict[component_type]

        component = Marker()
        component.header.frame_id = "%s/base_link"%self.agent_id
        component.ns = "%s__%s"%(self.agent_id, ns)
        component.id = marker_index

        component.type = getattr(component, component_dict['type'])
        component.action = getattr(component, component_dict['action'])

        component.pose.position = Point(position[0], position[1], position[2])

        q = component_dict['quat']
        quat = quaternion_from_euler(q[0],q[1],q[2])
        component.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        scale = component_dict['scale']
        component.scale = Vector3(scale[0], scale[1], scale[2])

        color = component_dict['color']
        if self.agent_color != '' and component_type in self.color_dict[self.agent_color]:
            color = self.color_dict[self.agent_color][component_type]
        component.color = ColorRGBA(color[0], color[1], color[2], color[3])

        component.frame_locked = component_dict['frame_locked']

        if component.type == component.TEXT_VIEW_FACING:
            component.text = self.agent_id

        elif component.type == component.MESH_RESOURCE:
            component.mesh_resource = component_dict['mesh_resource']

        return component

