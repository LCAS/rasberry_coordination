#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import tf, json
import geometry_msgs.msg
import bayes_people_tracker.msg
from rasberry_coordination.coordinator_tools import logmsg


class TFPublishers:

    @staticmethod
    def get_tf_convertor(msg):
        tf_source_type = {'geometry_msgs/Pose': TFPublishers.PoseTFConvertor,
                          'geometry_msgs/PoseStamped': TFPublishers.PoseStampedTFConvertor,
                          'gps_grouped_array': TFPublishers.GPSPositionsTFPublisher,
                          'static': TFPublishers.StaticTFConvertor}
        return tf_source_type[msg.tf_source_type](msg)


    class TFConvertor(object):
        def __init__(self, msg):
            self.id = msg.id

            # Specify tf details
            self.base_frame = self.id + "/base_link" if self.id else "base_link"
            self.tf_broadcaster = tf.TransformBroadcaster()

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

        def cycle_tf(self):
            if not self.pose: return
            m = [[round(v,2) for v in g] for g in self.pose]
            logmsg(category="rviz", id=self.id, msg="%s cycle_tf %s" % (self.source_type, str(m).replace('\n','')))
            self.tf_broadcaster.sendTransform((m[0][0], m[0][1], m[0][2]),
                                              (m[1][0], m[1][1], m[1][2], m[1][3]),
                                              rospy.Time.now(),
                                              self.base_frame,
                                              "map")


    class StaticTFConvertor(TFConvertor):
        def __init__(self, msg):
            super(TFPublishers.StaticTFConvertor, self).__init__(msg)
        def convert_to_array(self, msg):
            return msg


    class PoseTFConvertor(TFConvertor):
        def __init__(self, msg):
            super(TFPublishers.PoseTFConvertor, self).__init__(msg)
            self.pose_sub = rospy.Subscriber(self.source_topic, geometry_msgs.msg.Pose, self.pose_cb)
        def convert_to_array(self, msg):
            pos, ori = msg.position, msg.orientation
            return [[pos.x, pos.y, pos.z],[ori.x, ori.y, ori.z, ori.w]]


    class PoseStampedTFConvertor(TFConvertor):
        def __init__(self, msg):
            super(TFPublishers.PoseStampedTFConvertor, self).__init__(msg)
            self.pose_sub = rospy.Subscriber(self.source_topic, geometry_msgs.msg.PoseStamped, self.pose_cb)
        def convert_to_array(self, msg):
            pos, ori = msg.pose.position, msg.pose.orientation
            return [[pos.x, pos.y, pos.z],[ori.x, ori.y, ori.z, ori.w]]


    class GPSPositionsTFPublisher(TFConvertor):
        def __init__(self, msg):
            super(TFPublishers.GPSPositionsTFConvertor, self).__init__(msg)
            self.gps_sub = rospy.Subscriber(self.source_topic, bayes_people_tracker.msg.PeopleStamped, self.pose_cb)
        def convert_to_array(self, msg):
            person = [p.person for p in msg.people if p.person.name == self.id and p.person.position != geometry_msgs.msg.Point()]
            if len(person) < 1: return [[0,0,0],[0,0,0,1]]
            pos = person[0].position
            return [[pos.x, pos.y, pos.z],[0, 0, 0, 1]]
