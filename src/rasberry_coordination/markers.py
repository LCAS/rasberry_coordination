#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import rospy
import visualization_msgs.msg
import tf
from std_msgs.msg import ColorRGBA

class ThorvaldMarkerPublisher(object):
    """
    """
    def __init__(self, robot_name):
        """
        """
        self.robot_name = robot_name
        self._marker_array = visualization_msgs.msg.MarkerArray()
        self.frame_id = self.robot_name+"/base_link"
        self._create_markers()
        self.marker_pub = rospy.Publisher("/%s/vis" %(robot_name), visualization_msgs.msg.MarkerArray, queue_size=10)
        self.publish()

    def publish(self):
        """
        """
        time_now = rospy.get_rostime()
        for marker in self._marker_array.markers:
            marker.header.stamp = time_now
            marker.lifetime = rospy.Duration(2)
        self.marker_pub.publish(self._marker_array)

    def _create_markers(self):
        """
        """
        # wheels
        wheel_0_marker = self._wheel_marker("wheel0", 0, position=(0.65, 0.3, 0.15))
        wheel_1_marker = self._wheel_marker("wheel1", 1, position=(0.65, -0.3, 0.15))
        wheel_2_marker = self._wheel_marker("wheel2", 2, position=(-0.65, -0.3, 0.15))
        wheel_3_marker = self._wheel_marker("wheel3", 3, position=(-0.65, 0.3, 0.15))

        self._marker_array.markers.append(wheel_0_marker)
        self._marker_array.markers.append(wheel_1_marker)
        self._marker_array.markers.append(wheel_2_marker)
        self._marker_array.markers.append(wheel_3_marker)

        # towers
        tower_0_marker = self._tower_marker("tower0", 4, position=(0.6, 0.3, 0.45))
        tower_1_marker = self._tower_marker("tower1", 5, position=(0.6, -0.3, 0.45))
        tower_2_marker = self._tower_marker("tower2", 6, position=(-0.6, -0.3, 0.45))
        tower_3_marker = self._tower_marker("tower3", 7, position=(-0.6, 0.3, 0.45))

        self._marker_array.markers.append(tower_0_marker)
        self._marker_array.markers.append(tower_1_marker)
        self._marker_array.markers.append(tower_2_marker)
        self._marker_array.markers.append(tower_3_marker)

        # front - back pipes
        pipe_0_marker = self._front_back_pipe_marker("pipe0", 8, position=(0.0, 0.32, 0.5))
        pipe_1_marker = self._front_back_pipe_marker("pipe1", 9, position=(0.0, 0.32, 0.4))
        pipe_2_marker = self._front_back_pipe_marker("pipe2", 10, position=(0.0, -0.32, 0.5))
        pipe_3_marker = self._front_back_pipe_marker("pipe3", 11, position=(0.0, -0.32, 0.4))

        self._marker_array.markers.append(pipe_0_marker)
        self._marker_array.markers.append(pipe_1_marker)
        self._marker_array.markers.append(pipe_2_marker)
        self._marker_array.markers.append(pipe_3_marker)

        # left - right pipes
        pipe_4_marker = self._left_right_pipe_marker("pipe4", 12, position=(0.65, 0.0, 0.5))
        pipe_5_marker = self._left_right_pipe_marker("pipe5", 13, position=(0.65, 0.0, 0.4))
        pipe_6_marker = self._left_right_pipe_marker("pipe6", 14, position=(-0.65, 0.0, 0.5))
        pipe_7_marker = self._left_right_pipe_marker("pipe7", 15, position=(-0.65, 0.0, 0.4))

        self._marker_array.markers.append(pipe_4_marker)
        self._marker_array.markers.append(pipe_5_marker)
        self._marker_array.markers.append(pipe_6_marker)
        self._marker_array.markers.append(pipe_7_marker)

        # name
        name_marker = self._name_marker("name", 16, position=(0, 0, 0.7))
        self._marker_array.markers.append(name_marker)

    def _wheel_marker(self, wheel_id, marker_index, position):
        """
        """
        wheel_marker = visualization_msgs.msg.Marker()
        wheel_marker.header.frame_id = self.frame_id
        wheel_marker.ns = wheel_id
        wheel_marker.id = marker_index
        wheel_marker.type = visualization_msgs.msg.Marker.CYLINDER
        wheel_marker.action = visualization_msgs.msg.Marker.ADD
        wheel_marker.pose.position.x = position[0]
        wheel_marker.pose.position.y = position[1]
        wheel_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(3.14/2, 0., 0.)
        wheel_marker.pose.orientation.x = quat[0]
        wheel_marker.pose.orientation.y = quat[1]
        wheel_marker.pose.orientation.z = quat[2]
        wheel_marker.pose.orientation.w = quat[3]
        wheel_marker.scale.x = 0.3
        wheel_marker.scale.y = 0.3
        wheel_marker.scale.z = 0.2
        wheel_marker.color.r = 0.0
        wheel_marker.color.g = 0.0
        wheel_marker.color.b = 0.0
        wheel_marker.color.a = 1.0
        wheel_marker.frame_locked = True

        return wheel_marker

    def _tower_marker(self, tower_id, marker_index, position):
        """
        """
        tower_marker = visualization_msgs.msg.Marker()
        tower_marker.header.frame_id = self.frame_id
        tower_marker.ns = tower_id
        tower_marker.id = marker_index
        tower_marker.type = visualization_msgs.msg.Marker.CUBE
        tower_marker.action = visualization_msgs.msg.Marker.ADD
        tower_marker.pose.position.x = position[0]
        tower_marker.pose.position.y = position[1]
        tower_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(3.14/2, 0., 0.)
        tower_marker.pose.orientation.x = quat[0]
        tower_marker.pose.orientation.y = quat[1]
        tower_marker.pose.orientation.z = quat[2]
        tower_marker.pose.orientation.w = quat[3]
        tower_marker.scale.x = 0.3
        tower_marker.scale.y = 0.3
        tower_marker.scale.z = 0.2
        tower_marker.color.r = 1.0
        tower_marker.color.g = 1.0
        tower_marker.color.b = 1.0
        tower_marker.color.a = 1.0
        tower_marker.frame_locked = True

        return tower_marker

    def _front_back_pipe_marker(self, pipe_id, marker_index, position):
        """
        """
        pipe_marker = visualization_msgs.msg.Marker()
        pipe_marker.header.frame_id = self.frame_id
        pipe_marker.ns = pipe_id
        pipe_marker.id = marker_index
        pipe_marker.type = visualization_msgs.msg.Marker.CYLINDER
        pipe_marker.action = visualization_msgs.msg.Marker.ADD
        pipe_marker.pose.position.x = position[0]
        pipe_marker.pose.position.y = position[1]
        pipe_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(0.0, 3.14/2.0, 0.0)
        pipe_marker.pose.orientation.x = quat[0]
        pipe_marker.pose.orientation.y = quat[1]
        pipe_marker.pose.orientation.z = quat[2]
        pipe_marker.pose.orientation.w = quat[3]
        pipe_marker.scale.x = 0.05
        pipe_marker.scale.y = 0.05
        pipe_marker.scale.z = 1.2
        pipe_marker.color.r = 0.5
        pipe_marker.color.g = 0.5
        pipe_marker.color.b = 0.5
        pipe_marker.color.a = 1.0
        pipe_marker.frame_locked = True

        return pipe_marker

    def _left_right_pipe_marker(self, pipe_id, marker_index, position):
        """
        """
        pipe_marker = visualization_msgs.msg.Marker()
        pipe_marker.header.frame_id = self.frame_id
        pipe_marker.ns = pipe_id
        pipe_marker.id = marker_index
        pipe_marker.type = visualization_msgs.msg.Marker.CYLINDER
        pipe_marker.action = visualization_msgs.msg.Marker.ADD
        pipe_marker.pose.position.x = position[0]
        pipe_marker.pose.position.y = position[1]
        pipe_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(3.14/2.0, 0.0, 0.0)
        pipe_marker.pose.orientation.x = quat[0]
        pipe_marker.pose.orientation.y = quat[1]
        pipe_marker.pose.orientation.z = quat[2]
        pipe_marker.pose.orientation.w = quat[3]
        pipe_marker.scale.x = 0.05
        pipe_marker.scale.y = 0.05
        pipe_marker.scale.z = 0.6
        pipe_marker.color.r = 0.5
        pipe_marker.color.g = 0.5
        pipe_marker.color.b = 0.5
        pipe_marker.color.a = 1.0
        pipe_marker.frame_locked = True

        return pipe_marker

    def _name_marker(self, name_id, marker_index, position):
        """
        """
        name_marker = visualization_msgs.msg.Marker()
        name_marker.header.frame_id = self.frame_id
        name_marker.ns = name_id
        name_marker.id = marker_index
        name_marker.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
        name_marker.action = visualization_msgs.msg.Marker.ADD
        name_marker.pose.position.x = position[0]
        name_marker.pose.position.y = position[1]
        name_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(3.14/2.0, 0.0, 0.0)
        name_marker.pose.orientation.x = quat[0]
        name_marker.pose.orientation.y = quat[1]
        name_marker.pose.orientation.z = quat[2]
        name_marker.pose.orientation.w = quat[3]
        name_marker.scale.x = 0.05
        name_marker.scale.y = 0.05
        name_marker.scale.z = 0.4
        name_marker.color.r = 0.0
        name_marker.color.g = 0.0
        name_marker.color.b = 0.0
        name_marker.color.a = 1.0
        name_marker.text = self.robot_name
        name_marker.frame_locked = True

        return name_marker


class ColoredThorvaldMarkerPublisher(ThorvaldMarkerPublisher):

    def __init__(self, robot_name, color=''):
        """
        """

        # Format marker object
        self.robot_name = robot_name
        self._marker_array = visualization_msgs.msg.MarkerArray()
        self.frame_id = self.robot_name+"/base_link"
        self._create_markers()

        # Define colors associated to specific marker components
        color_swaps = {
          'label': ('pipe',                 'wheel',            'tower'),
          'red':   (ColorRGBA(.5, 0, 0, 1), ColorRGBA(0,0,0,1), ColorRGBA(1,0,0,1)),
          'green': (ColorRGBA( 0,.5, 0, 1), ColorRGBA(0,0,0,1), ColorRGBA(0,1,0,1)),
          'blue':  (ColorRGBA( 0, 0,.5, 1), ColorRGBA(0,0,0,1), ColorRGBA(0,0,1,1)),
          'black': (ColorRGBA( 0, 0, 0, 1), ColorRGBA(0,0,0,1), ColorRGBA(0,0,0,1)),
          'white': (ColorRGBA( 1, 1, 1, 1), ColorRGBA(0,0,0,1), ColorRGBA(1,1,1,1))
        } #TODO: transpose this

        # Assign color to respective marker component
        if color in color_swaps:
            for marker in self._marker_array.markers:
                starts = [i for i, s in enumerate(color_swaps['label']) if marker.ns.startswith(s)]
                # print("Setting: marker component(%s) to color %s" % (marker.ns,str(starts)))
                if starts:
                    marker.color = color_swaps[color][starts[0]]

        # Publish marker to rviz
        self.marker_pub = rospy.Publisher("/%s/vis" %(robot_name), visualization_msgs.msg.MarkerArray, queue_size=10)
        self.publish()


class HumanMarkerPublisher(object):
    """
    """
    def __init__(self, picker_id):
        """
        """
        self.picker_id = picker_id
        self._marker_array = visualization_msgs.msg.MarkerArray()
        self.frame_id = self.picker_id+"/base_link"
        self._create_markers()
        self.marker_pub = rospy.Publisher("/%s/vis" %(picker_id), visualization_msgs.msg.MarkerArray, queue_size=10)
        self.publish()

    def publish(self):
        """
        """
        time_now = rospy.get_rostime()
        for marker in self._marker_array.markers:
            marker.header.stamp = time_now
        self.marker_pub.publish(self._marker_array)

    def _create_markers(self):
        """
        """
        # legs
        left_leg_marker = self._leg_marker("left_leg", 0, (0, 0.05, 0.4))
        right_leg_marker = self._leg_marker("right_leg", 1, (0, -0.05, 0.4))

        self._marker_array.markers.append(left_leg_marker)
        self._marker_array.markers.append(right_leg_marker)

        # body
        body_marker = self._body_marker("body", 2, (0, 0, 1.05))
        self._marker_array.markers.append(body_marker)

        # arms
        left_arm_marker = self._arm_marker("left_arm", 3, (0, 0.15, 1.0))
        right_arm_marker = self._arm_marker("right_arm", 4, (0, -0.15, 1.0))

        self._marker_array.markers.append(left_arm_marker)
        self._marker_array.markers.append(right_arm_marker)

        # head
        head_marker = self._head_marker("head", 5, (0, 0, 1.40))
        self._marker_array.markers.append(head_marker)

        # name
        name_marker = self._name_marker("name", 6, (0, 0, 1.8))
        self._marker_array.markers.append(name_marker)

    def _leg_marker(self, leg_id, marker_index, position):
        """
        """
        leg_marker = visualization_msgs.msg.Marker()
        leg_marker.header.frame_id = self.frame_id
        leg_marker.ns = leg_id
        leg_marker.id = marker_index
        leg_marker.type = visualization_msgs.msg.Marker.CYLINDER
        leg_marker.action = visualization_msgs.msg.Marker.ADD
        leg_marker.pose.position.x = position[0]
        leg_marker.pose.position.y = position[1]
        leg_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        leg_marker.pose.orientation.x = quat[0]
        leg_marker.pose.orientation.y = quat[1]
        leg_marker.pose.orientation.z = quat[2]
        leg_marker.pose.orientation.w = quat[3]
        leg_marker.scale.x = 0.08
        leg_marker.scale.y = 0.08
        leg_marker.scale.z = 0.8
        leg_marker.color.r = 0.23
        leg_marker.color.g = 0.46
        leg_marker.color.b = 0.72
        leg_marker.color.a = 1.0
        leg_marker.frame_locked = True

        return leg_marker

    def _body_marker(self, body_id, marker_index, position):
        """
        """
        body_marker = visualization_msgs.msg.Marker()
        body_marker.header.frame_id = self.frame_id
        body_marker.ns = body_id
        body_marker.id = marker_index
        body_marker.type = visualization_msgs.msg.Marker.CYLINDER
        body_marker.action = visualization_msgs.msg.Marker.ADD
        body_marker.pose.position.x = position[0]
        body_marker.pose.position.y = position[1]
        body_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        body_marker.pose.orientation.x = quat[0]
        body_marker.pose.orientation.y = quat[1]
        body_marker.pose.orientation.z = quat[2]
        body_marker.pose.orientation.w = quat[3]
        body_marker.scale.x = 0.15
        body_marker.scale.y = 0.25
        body_marker.scale.z = 0.5
        body_marker.color.r = 1.0
        body_marker.color.g = 1.0
        body_marker.color.b = 1.0
        body_marker.color.a = 1.0
        body_marker.frame_locked = True

        return body_marker

    def _arm_marker(self, arm_id, marker_index, position):
        """
        """
        arm_marker = visualization_msgs.msg.Marker()
        arm_marker.header.frame_id = self.frame_id
        arm_marker.ns = arm_id
        arm_marker.id = marker_index
        arm_marker.type = visualization_msgs.msg.Marker.CYLINDER
        arm_marker.action = visualization_msgs.msg.Marker.ADD
        arm_marker.pose.position.x = position[0]
        arm_marker.pose.position.y = position[1]
        arm_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        arm_marker.pose.orientation.x = quat[0]
        arm_marker.pose.orientation.y = quat[1]
        arm_marker.pose.orientation.z = quat[2]
        arm_marker.pose.orientation.w = quat[3]
        arm_marker.scale.x = 0.05
        arm_marker.scale.y = 0.05
        arm_marker.scale.z = 0.55
        arm_marker.color.r = 0.99
        arm_marker.color.g = 0.87
        arm_marker.color.b = 0.92
        arm_marker.color.a = 1.0
        arm_marker.frame_locked = True

        return arm_marker

    def _head_marker(self, head_id, marker_index, position):
        """
        """
        head_marker = visualization_msgs.msg.Marker()
        head_marker.header.frame_id = self.frame_id
        head_marker.ns = head_id
        head_marker.id = marker_index
        head_marker.type = visualization_msgs.msg.Marker.SPHERE
        head_marker.action = visualization_msgs.msg.Marker.ADD
        head_marker.pose.position.x = position[0]
        head_marker.pose.position.y = position[1]
        head_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        head_marker.pose.orientation.x = quat[0]
        head_marker.pose.orientation.y = quat[1]
        head_marker.pose.orientation.z = quat[2]
        head_marker.pose.orientation.w = quat[3]
        head_marker.scale.x = 0.2
        head_marker.scale.y = 0.2
        head_marker.scale.z = 0.2
        head_marker.color.r = 0.99
        head_marker.color.g = 0.87
        head_marker.color.b = 0.92
        head_marker.color.a = 1.0
        head_marker.frame_locked = True

        return head_marker

    def _name_marker(self, name_id, marker_index, position):
        """
        """
        name_marker = visualization_msgs.msg.Marker()
        name_marker.header.frame_id = self.frame_id
        name_marker.ns = name_id
        name_marker.id = marker_index
        name_marker.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
        name_marker.action = visualization_msgs.msg.Marker.ADD
        name_marker.pose.position.x = position[0]
        name_marker.pose.position.y = position[1]
        name_marker.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(3.14/2.0, 0.0, 0.0)
        name_marker.pose.orientation.x = quat[0]
        name_marker.pose.orientation.y = quat[1]
        name_marker.pose.orientation.z = quat[2]
        name_marker.pose.orientation.w = quat[3]
        name_marker.scale.x = 0.05
        name_marker.scale.y = 0.05
        name_marker.scale.z = 0.4
        name_marker.color.r = 0.0
        name_marker.color.g = 0.0
        name_marker.color.b = 0.0
        name_marker.color.a = 1.0
        name_marker.text = self.picker_id
        name_marker.frame_locked = True

        return name_marker