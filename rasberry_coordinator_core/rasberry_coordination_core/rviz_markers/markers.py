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
    def __init__(self, msg, dicts):
        self.id = msg.id

        # Save local references to dictionaries
        self.structure_dict = dicts['structures']
        self.components_dict = dicts['components']

        # Construct placeholder for marker
        self.marker_array = MarkerArray()
        self.structure = msg.structure
        self.colour = [msg.colour.r, msg.colour.g, msg.colour.b, msg.colour.a]

        # Construct tf manager to update position of published marker
        self.tf = TFPublishers.get_tf_convertor(msg)


    def generate_marker_array(self):
        logmsg(category="rviz", id=self.id, msg="generating marker")
        marker_array = MarkerArray()
        if self.structure not in self.structure_dict:
            logmsg(category="rviz", msg="    | %s not found, generating short_robot_load_4"%self.structure)
            self.structure = 'short_robot_load_4'
        for component_type,items in self.structure_dict[self.structure].items():
            for i in items:
                c = self.get_component(component_type, ns=i[0], marker_index=i[1], position=i[2])
                marker_array.markers.append(c)
        self.marker_array = marker_array

    def get_component(self, component_type, ns, marker_index, position):
        component_dict = self.components_dict[component_type]

        component = Marker()
        component.header.frame_id = "%s/base_link"%self.id
        component.ns = "%s__%s"%(self.id, ns)
        component.id = marker_index

        component.type = getattr(component, component_dict['type'])
        component.action = getattr(component, component_dict['action'])

        component.pose.position = Point(position[0], position[1], position[2])

        q = component_dict['quat']
        quat = quaternion_from_euler(q[0],q[1],q[2])
        component.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        scale = component_dict['scale']
        component.scale = Vector3(scale[0], scale[1], scale[2])

        colour = component_dict['colour']
        if self.colour != '':
            print(self.colour)
            print(component_dict['colour_multiplier'])
            colour = [c*m for c,m in zip(self.colour, component_dict['colour_multiplier'])]
        component.color = ColorRGBA(colour[0], colour[1], colour[2], colour[3])

        component.frame_locked = component_dict['frame_locked']

        if component.type == component.TEXT_VIEW_FACING:
            component.text = self.id

        elif component.type == component.MESH_RESOURCE:
            component.mesh_resource = component_dict['mesh_resource']

        return component

