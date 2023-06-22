#!/usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date: 10/aug/2022
# ----------------------------------

import time

import tf2_ros

import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
#import bayes_people_tracker.msg


class TFPublishers:

    @staticmethod
    def get_tf_convertor(msg, ros2node):
        tf_source_type = {'geometry_msgs/Pose': TFPublishers.PoseTFConvertor,
                          'geometry_msgs/PoseStamped': TFPublishers.PoseStampedTFConvertor,
                          #'gps_grouped_array': TFPublishers.GPSPositionsTFConvertor,
                          'static': TFPublishers.StaticTFConvertor}
        return tf_source_type[msg.tf_source_type](msg, ros2node)


    class TFConvertor(object):
        def __init__(self, msg, ros2node):
            self.ros2node = ros2node
            self.id = msg.id

            # Specify tf details
            self.base_frame = self.id + "/base_link" if self.id else "base_link"
            self.tf_broadcaster = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(ros2node)

            # Specify subscriber details
            self.source_topic = msg.tf_source_topic
            self.source_type = msg.tf_source_type
            self.pose = [[0,0,0],[0,0,0,1]]
            if msg.pose and msg.pose != geometry_msgs.msg.Pose():
                pos, ori = msg.pose.position, msg.pose.orientation
                self.pose = [[pos.x, pos.y, pos.z],[ori.x, ori.y, ori.z, ori.w]]

        def convert_to_array(self, msg):
            return [[0,0,0],[0,0,0,1]]

        def pose_cb(self, msg):
            self.pose = self.convert_to_array(msg)
            self.cycle_tf()

        def cycle_tf(self):
            if not self.pose: return
            m = [[round(v,2) for v in g] for g in self.pose]
            print("%s %s cycle_tf %s" % (self.id, self.source_type, str(m).replace('\n','')))

            t = TransformStamped()
            t.header.stamp = self.ros2node.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(m[0][0])
            t.transform.translation.y = float(m[0][1])
            t.transform.translation.z = float(m[0][2])
            t.transform.rotation.x = float(m[1][0])
            t.transform.rotation.y = float(m[1][1])
            t.transform.rotation.z = float(m[1][2])
            t.transform.rotation.w = float(m[1][3])

            self.tf_broadcaster.sendTransform(t)

    class StaticTFConvertor(TFConvertor):
        def __init__(self, msg, ros2node):
            super(TFPublishers.StaticTFConvertor, self).__init__(msg, ros2node)
        def convert_to_array(self, msg):
            return msg


    class PoseTFConvertor(TFConvertor):
        def __init__(self, msg, ros2node):
            super(TFPublishers.PoseTFConvertor, self).__init__(msg, ros2node)
            self.pose_sub = self.ros2node.create_subscription(geometry_msgs.msg.Pose, self.source_topic, self.pose_cb, 10)
        def convert_to_array(self, msg):
            pos, ori = msg.position, msg.orientation
            return [[pos.x, pos.y, pos.z],[ori.x, ori.y, ori.z, ori.w]]


    class PoseStampedTFConvertor(TFConvertor):
        def __init__(self, msg, ros2node):
            super(TFPublishers.PoseStampedTFConvertor, self).__init__(msg, ros2node)
            self.pose_sub = self.ros2node.create_subscription(geometry_msgs.msg.PoseStamped, self.source_topic, self.pose_cb)
        def convert_to_array(self, msg):
            pos, ori = msg.pose.position, msg.pose.orientation
            return [[pos.x, pos.y, pos.z],[ori.x, ori.y, ori.z, ori.w]]


#    class GPSPositionsTFConvertor(TFConvertor):
#        def __init__(self, msg, ros2node):
#            super(TFPublishers.GPSPositionsTFConvertor, self).__init__(msg, ros2node)
#            self.gps_sub = self.ros2node.create_subscription(bayes_people_tracker.msg.PeopleStamped, self.source_topic, self.pose_cb)
#        def convert_to_array(self, msg):
#            person = [p.person for p in msg.people if p.person.name == self.id and p.person.position != geometry_msgs.msg.Point()]
#            if len(person) < 1: return [[0,0,0],[0,0,0,1]]
#            pos = person[0].position
#            return [[pos.x, pos.y, pos.z],[0, 0, 0, 1]]

