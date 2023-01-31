#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import actionlib
import rospy
import tf
import traceback
#from random import random
from rospy import Publisher, Subscriber
from rospy_message_converter.message_converter import convert_dictionary_to_ros_message as rosmsg

#from topological_navigation.tmap_utils import get_node as get_topomap_node, get_distance_node_pose_from_tmap2 as node_pose_dist
#from topological_navigation.route_search import TopologicalRouteSearch
from std_msgs.msg import Header, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from topological_navigation_msgs.msg import GotoNodeGoal, GotoNodeAction, ClosestEdges
from strands_navigation_msgs.msg import ExecutePolicyModeGoal, ExecutePolicyModeAction, TopologicalMap, TopologicalRoute

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.task_management.modules.navigation.interfaces.GeneralNavigator import GeneralNavigator


class RobotDebug(GeneralNavigator):
    def __init__(self, agent, details):
        super(RobotDebug, self).__init__(agent, details)
        aid = self.agent.agent_id

        # Containers
        self.buffer = []
        self.execpolicy_goal = ExecutePolicyModeGoal()

        # Set step-delay param
        param = '/rasberry_coordination/task_modules/navigation/debug_robot_step_delay'
        default = rospy.get_param(param, 0.5)

        # Route Publishers
        goal_topic = "/%s/topological_navigation/execute_policy_mode/goal" % aid
        self.goal_publisher  = Publisher(goal_topic, ExecutePolicyModeGoal, latch=True, queue_size=5)
        self.goal_subscriber = Subscriber(goal_topic, ExecutePolicyModeGoal, self.subgoal)

        # To update robots current location
        self.current_node_pub = Publisher("/%s/current_node" % aid, String, latch=True, queue_size=5)
        self.closest_node_pub = Publisher("/%s/closest_node" % aid, String, latch=True, queue_size=5)
        self.closest_edge_pub = Publisher("/%s/closest_edges" % aid, ClosestEdges, latch=True, queue_size=5)

        # RVIZ display tools
        self.rviz_route_publisher = Publisher("/%s/current_route" % aid, Path, latch=True, queue_size=5)
        #self.rviz_route_progress_publisher = Publisher("/%s/current_route_remaining" % aid, Path, latch=True, queue_size=5)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.pose_publisher = Publisher("/%s/robot_pose" % aid, Pose, latch=True, queue_size=5)


    def set_execpolicy_goal(self, goal):
        self.setgoal(goal)

    def cancel_execpolicy_goal(self):
        self.cangoal()


    """
cangoal()
  empty buffer

setgoal()
  buffer()
  pubgoal()

pubgoal()
  !buffer
    publish goal()

subgoal()
  telemove()
  wait()
  teleport()
  filter()
  pubgoal()

    """

    def cangoal(self):
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='*) CANCEL')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | []')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | []')
        self.execpolicy_goal, self.buffer = None, []

    def setgoal(self, goal):
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='0) SET')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(goal.route.source).replace('\n',''))
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(goal.route.edge_id).replace('\n',''))

        # Replace current node with target node in goal source
        if goal.route.source:
            goal.route.source = goal.route.source[1:]+[goal.route.edge_id[-1].split('_')[1]]
            logmsg(category='vr_roc', id=self.agent.agent_id, msg='as')
            logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(goal.route.source).replace('\n',''))
            logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(goal.route.edge_id).replace('\n',''))

        # Remove current edge if currently at when interrupted
        if self.agent.location.closest_edge and self.agent.location.closest_edge == goal.route.edge_id[0]:
            goal.route.edge_id.pop(0)
            logmsg(category='vr_roc', id=self.agent.agent_id, msg='as')
            logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(goal.route.source).replace('\n',''))
            logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(goal.route.edge_id).replace('\n',''))

        self.buffer.append(goal)
        self.pubgoal()

    def pubgoal(self):
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='1) Pubgoal')
        if self.buffer:
            self.execpolicy_goal, self.buffer = self.buffer[-1], []
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(self.execpolicy_goal.route.source).replace('\n',''))
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(self.execpolicy_goal.route.edge_id).replace('\n',''))
        self.goal_publisher.publish(self.execpolicy_goal)
        self.publish_path()

    def subgoal(self, msg):
        try:
            print("\n\n")
            logmsg(category='vr_rob', id=self.agent.agent_id, msg='2) Subgoal')

            # Move to next edge in route
            if self.execpolicy_goal.route.edge_id:
                nxt = self.execpolicy_goal.route.source[-1] if self.execpolicy_goal.route.source else ''
                logmsg(category='vr_rob', id=self.agent.agent_id, msg='2) Move to edge: %s'%nxt)
                logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(self.execpolicy_goal.route.edge_id).replace('\n',''))
                self.wait()
                self.telemove()
                self.filter_edge()

            # Move to next node in route
            if self.execpolicy_goal.route.source:
                nxt = self.execpolicy_goal.route.source[-1] if self.execpolicy_goal.route.source else ''
                logmsg(category='vr_rob', id=self.agent.agent_id, msg='2) Move to node: %s'%nxt)
                logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(self.execpolicy_goal.route.source).replace('\n',''))
                self.wait()
                self.teleport()
                self.filter_node()

            # Publish remainder of route for next cycle
            logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(self.execpolicy_goal.route.source).replace('\n',''))
            logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | %s'%str(self.execpolicy_goal.route.edge_id).replace('\n',''))
            if self.execpolicy_goal.route.source:
                self.pubgoal()
        except:
            print(traceback.format_exc())
            pass

    """ --------------- UTILS 1 ------------- """

    """
wait()

telemove()
  publish edge_tf
  publish edge_pose
  publish edge

teleport()
  publish tf
  publish pose
  publish node
  publish path

filter()
    """

    def wait(self):
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='    | i) Wait')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | %s'%str(self.execpolicy_goal.route.source).replace('\n',''))
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | %s'%str(self.execpolicy_goal.route.edge_id).replace('\n',''))
        rospy.sleep(rospy.get_param('/rasberry_coordination/task_modules/navigation/debug_robot_step_delay', 2)/2)
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='    | i) Wait End')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | %s'%str(self.execpolicy_goal.route.source).replace('\n',''))
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | %s'%str(self.execpolicy_goal.route.edge_id).replace('\n',''))

    def telemove(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | ii) Telemove')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | %s'%str(self.execpolicy_goal.route.edge_id).replace('\n',''))
        #self.publish_edge_tf()
        self.publish_edge_pose()
        self.publish_edge_name()

    def teleport(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | ii) Teleport')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | %s'%str(self.execpolicy_goal.route.source).replace('\n',''))
        #self.publish_node_tf()
        self.publish_node_pose()
        self.publish_node_name()
        self.publish_path()

    def filter_edge(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='    | iii) FilterEdge')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | %s'%str(self.execpolicy_goal.route.edge_id).replace('\n',''))
        self.execpolicy_goal.route.edge_id.pop(0)

    def filter_node(self):
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='    | iii) FilterNode')
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | %s'%str(self.execpolicy_goal.route.source).replace('\n',''))
        self.execpolicy_goal.route.source.pop(0)

    """ --------------- UTILS 2 ------------- """

    """
publish_node_tf()
publish_edge_tf()

publish_node_pose()
publish_edge_pose()

publish_node_name()
publish_edge_name()

publish_path()
    """


    def publish_node_tf(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | a) Pub Node TF')
        pos, ori = self.agent.map_handler.get_node_tf(self.execpolicy_goal.route.source[0])
        tim = rospy.Time.now()
        link = "%s/base_link" % self.agent.agent_id
        print(pos)
        print(ori)
        self.tf_broadcaster.sendTransform(pos, ori, tim, link, "map")
    def publish_edge_tf(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | a) Pub Edge TF')
        n1, n2 = self.execpolicy_goal.route.edge_id[0].split('_')
        pos1, ori = self.agent.map_handler.get_node_tf(n1)
        pos2, _   = self.agent.map_handler.get_node_tf(n2)
        pos = tuple([a+b/2 for a, b in zip(pos1,pos2)])
        tim = rospy.Time.now()
        link = "%s/base_link" % self.agent.agent_id
        print(pos)
        print(ori)
        self.tf_broadcaster.sendTransform(pos, ori, tim, link, "map")


    def publish_node_pose(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | b) Pub Node Pose')
        pos, ori = self.agent.map_handler.get_node_tf(self.execpolicy_goal.route.source[0])
        pose = Pose(Point(x=pos[0],y=pos[1],z=pos[2]), Quaternion(x=ori[0],y=ori[1],z=ori[2],w=ori[3]))
        self.pose_publisher.publish(pose)
    def publish_edge_pose(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | b) Pub Edge Pose')
        n1, n2 = self.execpolicy_goal.route.edge_id[0].split('_')
        pos1, ori = self.agent.map_handler.get_node_tf(n1)
        pos2, _   = self.agent.map_handler.get_node_tf(n2)
        pos = [(a+b)/2 for a, b in zip(pos1,pos2)]
        pose = Pose(Point(x=pos[0],y=pos[1],z=pos[2]), Quaternion(x=ori[0],y=ori[1],z=ori[2],w=ori[3]))
        self.pose_publisher.publish(pose)


    def publish_node_name(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | c) Pub Node Name')
        node = self.execpolicy_goal.route.source[0]
        self.current_node_pub.publish(node)
        self.closest_node_pub.publish(node)
    def publish_edge_name(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | c) Pub Edge Name')
        self.current_node_pub.publish("none")
        edge = self.execpolicy_goal.route.edge_id[0]
        edges = ClosestEdges(edge_ids=[edge], distances=[0.0])
        self.closest_edge_pub.publish(edges)


    def publish_path(self):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='        | d) Pub Path')
        route = Path()
        route.header.frame_id = "map"
        if self.execpolicy_goal.route.source:
            route.poses = [self.get_pose(self.agent.location())]
            route.poses += [self.get_pose(node) for node in self.execpolicy_goal.route.source]
        self.rviz_route_publisher.publish(route)


    """ --------------- UTILS 3 ------------- """


    def get_pose(self, node):
        POSE = PoseStamped()
        POSE.header.frame_id = "map"
        pose = self.get_node(node)['node']['pose']
        p, o = pose['position'], pose['orientation']
        POSE.pose.position = Point(x=p['x'], y=p['y'], z=p['z'])
        POSE.pose.orientation = Quaternion(x=o['x'], y=o['y'], z=o['z'], w=o['w'])
        return POSE


    def get_node(self, node):
        return self.agent.map_handler.get_node(node)


    """ ------------------------------------- """





    #
    # A 2 B 2 C 2 D 2 E
    #
    # A 2 B 2 c 2         <- this far in
    #         c 2 D ...   <- routing begins thinking we at C
    #         C 2 D 2 E   <- new route includes the node we past and the node we were jus waiting for
    #         -   D 2 E   <- we need to delete the node left and move to D
    #
    #
    # A 2 B 2 C 2 d 2     <- this far in faking D
    #             D       <- routing begins thinking we at d
    #             D 2 E   <- new route includes the node we at
    #             D 2 E   <- but we alredy ther, so all good
    #                      ^ but by faking d, stage completes
    #
    #
    # A 2 B 2 c 2         <- this far in
    #         c           <- routing begins thinking we at C
    # A 2 B 2 C 2 d 2     <- we complete move to D
    #         c 2 D 2 E   <- new route includes C and D
    #         -   D 2 E   <- we need to delete C
    #             D 2 E   <- we already at D
    #         -   -   E   <- so we need to delete D

