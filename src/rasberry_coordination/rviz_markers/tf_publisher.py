#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------


import rospy
import tf, json
import geometry_msgs.msg
from bayes_people_tracker.msg import PeopleStamped
from rasberry_coordination.coordinator_tools import logmsg


class TFPublishers:

    @staticmethod
    def get_tf_convertor(agent_id, agent_type, source):
        tf_source_type = {'short_robot': TFPublishers.PoseTFConvertor,
                          'tall_robot': TFPublishers.PoseTFConvertor,
                          'human_sim': TFPublishers.PoseStampedTFConvertor,
                          'human': TFPublishers.GPSPositionsTFPublisher,
                          'special_node': TFPublishers.StaticTFConvertor}
        b = [k for k in tf_source_type.keys() if agent_type.startswith(k)][0]
        return tf_source_type[b](agent_id, source)


    class StaticTFConvertor(object):
        def __init__(self, agent_id, source, log=False):
            self.log = log
            self.agent_id = agent_id

            if agent_id == "":
               self.base_frame = "base_link"
            else:
                self.base_frame = self.agent_id + "/base_link"
            self.tf_broadcaster = tf.TransformBroadcaster()

            self.last_message = json.loads(source) if source else None

        def cycle_tf(self):
            msg = self.last_message
            if self.log:
                rospy.loginfo(msg.pose)
            logmsg(category="rviz", id=self.agent_id, msg="static cycle_tf %s" % msg)
            if not msg: return
            self.tf_broadcaster.sendTransform((msg[0][0], msg[0][1], msg[0][2]),
                                              (msg[1][0], msg[1][1], msg[1][2], msg[1][3]),
                                              rospy.Time.now(),
                                              self.base_frame,
                                              "map")


    class PoseTFConvertor(object):
        def __init__(self, agent_id, source, log=False):
            self.log = log
            self.agent_id = agent_id

            if agent_id == "":
               self.base_frame = "base_link"
            else:
                self.base_frame = self.agent_id + "/base_link"
            self.tf_broadcaster = tf.TransformBroadcaster()

            self.last_message = None
            self.pose_sub = rospy.Subscriber(source, geometry_msgs.msg.Pose, self.pose_cb)

        def cycle_tf(self):
            if self.last_message:
                self.pose_cb(self.last_message)

        def pose_cb(self, msg):
            self.last_message = msg
            if self.log:
                rospy.loginfo(msg.pose)
            logmsg(category="rviz", id=self.agent_id, msg="static cycle_tf %s" % str(msg).replace('\n',''))
            self.tf_broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                                              (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                                              rospy.Time.now(),
                                              self.base_frame,
                                              "map")


    class PoseStampedTFConvertor(object):
        def __init__(self, agent_id, source, log=False):
            self.log = log
            self.agent_id = agent_id

            self.last_message = None
            self.pose_sub = rospy.Subscriber(source, geometry_msgs.msg.PoseStamped, self.pose_cb)

            if agent_id == "":
                self.base_frame = "base_link"
            else:
                self.base_frame = self.agent_id + "/base_link"
            self.tf_broadcaster = tf.TransformBroadcaster()

        def cycle_tf(self):
            if self.last_message:
                self.pose_cb(self.last_message)

        def pose_cb(self, msg):
            self.last_message = msg
            if self.log:
                rospy.loginfo(msg.pose)
            self.tf_broadcaster.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                                              (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
                                              rospy.Time.now(),
                                              self.base_frame,
                                              "map")


    class GPSPositionsTFPublisher(object):
        def __init__(self, agent_id, source, log=False):
            self.log = log
            self.agent_id = agent_id
            self.last_message = None
            self.gps_sub = rospy.Subscriber(source, PeopleStamped, self.gps_cb)
            if agent_id == "":
                self.base_frame = "base_link"
            else:
                self.base_frame = self.agent_id + "/base_link"
            self.tf_broadcaster = tf.TransformBroadcaster()
            print("using: %s"%source_topic)
            print("broadcaster to use %s base frame"%self.base_frame)

        def cycle_tf(self):
            if self.last_message:
                print('cycling tf (gps)')
                self.gps_cb(self.last_message)

        def gps_cb(self, msg):
            self.last_message = msg
            print("gps_cb")
            person = [p.person for p in msg.people if p.person.name == self.agent_id and p.person.position != geometry_msgs.msg.Point()]
            print(person)
            if len(person) < 1: return
            print("gps2")
            person = person[0]
            print(person)

            if self.log:
                rospy.loginfo(msg.pose)
            self.tf_broadcaster.sendTransform((p.person.position.x, p.person.position.y, p.person.position.z),
                                              (0, 0, 0, 1),
                                              rospy.Time.now(),
                                              self.base_frame,
                                              "map")

