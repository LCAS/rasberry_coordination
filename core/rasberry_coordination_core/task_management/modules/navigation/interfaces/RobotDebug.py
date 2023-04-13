# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import time
import rclpy
import tf2_ros
import traceback

# Messages
from std_msgs.msg import Header, String
from nav_msgs.msg import Path
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from topological_navigation_msgs.msg import ClosestEdges, CurrentEdge, ExecutePolicyModeGoal, NavRoute

# Components
from rasberry_coordination_core.task_management.modules.base.interfaces.Interface import iFACE as Interface
from rasberry_coordination_core.task_management.modules.navigation.interfaces.GeneralNavigator import iFACE as GeneralNavigator
from rasberry_coordination_core.task_management.__init__ import Stages
from rasberry_coordination_core.task_management.containers.Task import TaskObj as Task
from rasberry_coordination_core.task_management.__init__ import fetch_property

# ROS2
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup as RCG
from rasberry_coordination_core.node import GlobalNode

# Logging
from rasberry_coordination_core.utils.logmsg import logmsg


# Automanaged by rasberry_coordination_core.task_management.__init__.load_modules
# Interface class must be named `iFACE` to be recognised for import
# It will then be identifiable by its Interfaces[<<module>>][<<filename>>]
class iFACE(GeneralNavigator):
    def __init__(self, agent, details):
        super(iFACE, self).__init__(agent, details)
        aid = self.agent.agent_id

        # Containers
        self.buffer = []
        self.execpolicy_goal = ExecutePolicyModeGoal()

        # Set step-delay param
        param = '~/task_modules/navigation/debug_robot_step_delay'
        default = fetch_property(param, 0.5)

        # Route Publishers
        goal_topic = f"/{aid}/topological_navigation/execute_policy_mode/goal"
        self.goal_publisher  = GlobalNode.create_publisher(ExecutePolicyModeGoal, goal_topic, 2)
        self.goal_subscriber = GlobalNode.create_subscription(ExecutePolicyModeGoal, goal_topic, self.subgoal1, 10, callback_group=RCG())

        # RVIZ display tools
        self.rviz_route_publisher = GlobalNode.create_publisher(Path, f"/{aid}/current_route", 1)
        self.tf_broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(GlobalNode)
        self.pose_publisher = GlobalNode.create_publisher(Pose, f"/{aid}/robot_pose", 1)

        # Location Updating
        self.use_topic_publishing = False
        self.closest_node_pub = GlobalNode.create_publisher(String, f"/{aid}/closest_node", 1)
        self.current_node_pub = GlobalNode.create_publisher(String, f"/{aid}/current_node", 1)
        self.closest_edges_pub = GlobalNode.create_publisher(ClosestEdges, f"/{aid}/closest_edges", 1)
        self.current_edge_pub = GlobalNode.create_publisher(CurrentEdge, f"/{aid}/current_edge", 1)

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
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='Execpolicy Goal Cancelled')
        self.execpolicy_goal, self.buffer = None, []

    def setgoal(self, goal):
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='Execpolicy Goal Created')

        # Filter route to prevent navigating to already at/left locations
        # Remove first node/edge if agent is already there
        loc = self.agent.location
        if goal.route.source and goal.route.source[0] in [loc.current_node, loc.previous_node]:
            goal.route.source.pop(0)
        if goal.route.edge_id and goal.route.edge_id[0] in [loc.closest_edge]:
            goal.route.edge_id.pop(0)

        # Display filtered routes
        self.path_gen(goal.route)
        self.buffer.append(goal)
        self.pubgoal()

    def pubgoal(self):
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='New Execpolicy Goal Published')
        if self.buffer:
            self.execpolicy_goal, self.buffer = self.buffer[-1], []

        # Print route details
        if self.execpolicy_goal.route == NavRoute(): return

        # Print route details
        self.path_gen(self.execpolicy_goal.route)

        # Publish new path to rviz
        self.publish_path(self.execpolicy_goal.route)

        # Publish route to execute
        if self.use_topic_publishing:
            self.goal_publisher.publish(self.execpolicy_goal)
        else:
            self.subgoal1(self.execpolicy_goal)

    """
#    def subgoal(self, msg):
#        try:
#            aid = self.agent.agent_id
#            if not self.execpolicy_goal.route.source and not self.execpolicy_goal.route.edge_id:
#                return
#            print("\n\n")
#
#            logmsg(category='vr_rob', id=aid, msg='Execpolicy Goal:')
#            logmsg(category='vr_rob', id=aid, msg='   | Route:')
#            self.path_gen(self.execpolicy_goal.route, format='   :   | %s')
#
#
#            # Move to next edge in route unless next node is not reached
#            goal = self.execpolicy_goal
#            if goal and goal.route.edge_id and goal.route.source and not goal.route.edge_id[0].startswith(goal.route.source[0]):
#                edge = goal.route.edge_id[0]
#
#                # Move to start of edge
#                logmsg(category='vr_rob', id=aid, msg='Moving to edge start: %s'%edge)
#                self.wait(percentage=1/3)
#                self.telemove(edge, percentage=1/3)
#
#                # Move to end of edge
#                logmsg(category='vr_rob', id=aid, msg='Moving to edge end: %s'%edge)
#                self.wait(percentage=1/3)
#                self.telemove(edge, percentage=2/3)
#
#                # Display remaining route
#                logmsg(category='vr_rod', id=aid, msg='   | Remaining Route:')
#                self.filter_edge(edge)
#                self.path_gen(goal.route, format='   :   | %s')
#
#
#            # Move to next node in route
#            goal = self.execpolicy_goal
#            if goal and goal.route.source:
#                node = goal.route.source[0]
#
#                # Move to node
#                logmsg(category='vr_rob', id=aid, msg='Moving to node: %s'%node)
#                self.wait(percentage=1/3)
#                self.teleport(node)
#
#                # Display remaining route
#                logmsg(category='vr_rod', id=aid, msg='   | Remaining Route:')
#                self.filter_node(node)
#                self.path_gen(goal.route, format='   :   | %s')
#
#
#            # Publish remainder of route for next cycle
#            goal = self.execpolicy_goal
#            if goal:
#                self.publish_path(goal.route)
#                self.pubgoal()
#
#        except:
#            print(traceback.format_exc())
#            pass
"""

###################################################################################################
###################################################################################################

    def subgoal1(self, msg):
        # Check goal is valid
        if not self.execpolicy_goal.route.source and not self.execpolicy_goal.route.edge_id:
            return
        print("\n\n")
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='Execpolicy Goal:')
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='   | Route:')
        self.path_gen(self.execpolicy_goal.route, format='   :   | %s')

        # Begin traversal onto next edge (unless next node is already reached)
        goal = self.execpolicy_goal
        if goal and goal.route.edge_id and goal.route.source and not goal.route.edge_id[0].startswith(goal.route.source[0]):
            edge = goal.route.edge_id[0]

            # Begin navigation to start of edge
            logmsg(category='vr_rob', id=self.agent.agent_id, msg='Moving to edge start: %s'%edge)
            delay = self.wait(percentage=1/3)
            self.subgoal1_timer = GlobalNode.create_timer(delay, self.subgoal2)
            return
        else:
            self.subgoal2()

    def subgoal2(self):
        self.subgoal1_timer.cancel()
        # Begin traversal onto next edge (unless next node is already reached)
        goal = self.execpolicy_goal
        if goal and goal.route.edge_id and goal.route.source and not goal.route.edge_id[0].startswith(goal.route.source[0]):
            edge = goal.route.edge_id[0]

            # Complete move to start of edge
            self.telemove(edge, percentage=1/3)

            # Begin navigation to end of edge
            logmsg(category='vr_rob', id=self.agent.agent_id, msg='Moving to edge end: %s'%edge)
            delay = self.wait(percentage=1/3)
            self.subgoal2_timer = GlobalNode.create_timer(delay, self.subgoal3)
            return
        else:
            self.subgoal3()

    def subgoal3(self):
        self.subgoal2_timer.cancel()
        # Complete traversal to end of edge (unless next node is already reached)
        goal = self.execpolicy_goal
        if goal and goal.route.edge_id and goal.route.source and not goal.route.edge_id[0].startswith(goal.route.source[0]):
            edge = goal.route.edge_id[0]

            # Complete move to end of edge
            self.telemove(edge, percentage=2/3)

            # Display remaining route
            logmsg(category='vr_rod', id=self.agent.agent_id, msg='   | Remaining Route:')
            self.filter_edge(edge)
            self.path_gen(goal.route, format='   :   | %s')

        # Move to next node in route
        goal = self.execpolicy_goal
        if goal and goal.route.source:
            node = goal.route.source[0]

            # Begin navigation to node
            logmsg(category='vr_rob', id=self.agent.agent_id, msg='Moving to node: %s'%node)
            delay = self.wait(percentage=1/3)
            self.subgoal3_timer = GlobalNode.create_timer(delay, self.subgoal4)
            return
        else:
            self.subgoal4()

    def subgoal4(self):
        self.subgoal3_timer.cancel()

        # Move to next node in route
        goal = self.execpolicy_goal
        if goal and goal.route.source:
            node = goal.route.source[0]

            # Complete navigation to node
            self.teleport(node)

            # Display remaining route
            logmsg(category='vr_rod', id=self.agent.agent_id, msg='   | Remaining Route:')
            self.filter_node(node)
            self.path_gen(goal.route, format='   :   | %s')

        # Publish remainder of route for next cycle
        goal = self.execpolicy_goal
        if goal:
            self.publish_path(goal.route)
            self.pubgoal()

###################################################################################################
###################################################################################################
###################################################################################################
###################################################################################################
###################################################################################################

    """ --------------- UTILS 1 ------------- """

    """
    wait()

    telemove()
      publish edge_tf
      publish edge_pose
      publish edge_name

    teleport()
      publish node_tf
      publish node_pose
      publish node_name
      publish path

    filter()
    """

    def wait(self, percentage=1/2):
        #if self.details['smart_delay']: delay = get_smart_travel_time(route.edge) / 2
        delay = percentage * float(fetch_property('navigation', 'debug_robot_step_delay', 2).double_value)
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='   | Travel time: %s seconds' % round(delay,3))

        #time.sleep(delay) # doesnt allow background processes to continue

        #rclpy.spin_once(GlobalNode, timeout_sec=delay) # doesnt allow executor.spin() to take back control

        #GlobalNode.create_timer(delay, self.delayed_callback_fn) # doesnt allow callback to proceed here

        #await asyncio.sleep(delay) # not supported alongside ros2

        #GlobalNode.create_timer(delay, self.end_wait) # i dont want to have multiple callbacks

        #GlobalNode.loop_rate.sleep(delay)
        #GlobalNode.create_rate(delay, GlobalNode.get_clock()).sleep() # rate hz is not seconds

        #from rclpy.clock import ROSClock
        #from rclpy.duration import Duration
        #clock = ROSClock().sleep_for(Duration(seconds=delay)) # stalls main thread
        #assert ROSClock().sleep_for(Duration(seconds=delay)) # stalls main thread
        return delay


    def telemove(self, edge, percentage=1/2):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='   | Arrived at Edge')
        self.publish_edge_pose(edge, percentage)
        self.publish_edge_name(edge, percentage)

    def teleport(self, node):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='   | Arrived at Node')
        self.publish_node_pose(node)
        self.publish_node_name(node)

    def filter_edge(self, edge):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='   | Route Edge Popped')
        if not self.execpolicy_goal or self.execpolicy_goal.route.edge_id[0] != edge:
            aid = self.agent.agent_id
            logmsg(level='error', category='vr_roc', id=aid, msg='   :   | Edge is not in route, has route changed?')
            return
        self.execpolicy_goal.route.edge_id.pop(0)

    def filter_node(self, node):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='   | Route Node Popped')
        if not self.execpolicy_goal or self.execpolicy_goal.route.source[0] != node:
            aid = self.agent.agent_id
            logmsg(level='error', category='vr_roc', id=aid, msg='   :   | Node is not in route, has route changed?')
            return
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


    def publish_node_tf(self, node):
        #logmsg(category='vr_roc', id=self.agent.agent_id, msg='   :   | a) Pub Node TF')
        pos, ori = self.agent.map_handler.get_node_tf(node)
        tim = time.time()
        link = "%s/base_link" % self.agent.agent_id
        self.tf_broadcaster.sendTransform(pos, ori, tim, link, "map")
    def publish_edge_tf(self, edge, percentage):
        #logmsg(category='vr_roc', id=self.agent.agent_id, msg='   :   | a) Pub Edge TF')
        old, new = edge.split('_')
        pos1, ori = self.agent.map_handler.get_node_tf(old)
        pos2, _   = self.agent.map_handler.get_node_tf(new)
        pos = tuple([a+(b-a)*percentage for a, b in zip(pos1,pos2)])
        tim = time.time()
        link = "%s/base_link" % self.agent.agent_id
        self.tf_broadcaster.sendTransform(pos, ori, tim, link, "map")


    def publish_node_pose(self, node):
        #logmsg(category='vr_roc', id=self.agent.agent_id, msg='   :   | b) Pub Node Pose')
        pos, ori = self.agent.map_handler.get_node_tf(node)
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = ori[0]
        pose.orientation.y = ori[1]
        pose.orientation.z = ori[2]
        pose.orientation.w = ori[3]
        self.pose_publisher.publish(pose)
    def publish_edge_pose(self, edge, percentage=2/3):
        #logmsg(category='vr_roc', id=self.agent.agent_id, msg='   :   | b) Pub Edge Pose')
        old, new = edge.split('_')
        pos1, ori = self.agent.map_handler.get_node_tf(old)
        pos2, _   = self.agent.map_handler.get_node_tf(new)
        pos = [a+(b-a)*percentage for a, b in zip(pos1,pos2)]
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = ori[0]
        pose.orientation.y = ori[1]
        pose.orientation.z = ori[2]
        pose.orientation.w = ori[3]
        self.pose_publisher.publish(pose)


    def publish_node_name(self, node):
        #logmsg(category='vr_roc', id=self.agent.agent_id, msg='   :   | c) Pub Node Name')
        if self.use_topic_publishing:
            self.closest_node_pub.publish(String(data=node))
            self.current_node_pub.publish(String(data=node))
            self.closest_edges_pub.publish(ClosestEdges())
            self.current_edge_pub.publish(CurrentEdge())
        else:
            self.agent.location.closest_node_cb(String(data=node))
            self.agent.location.current_node_cb(String(data=node))
            self.agent.location.closest_edges_cb(ClosestEdges())
            self.agent.location.current_edge_cb(CurrentEdge())

    def publish_edge_name(self, edge, percentage):
        #logmsg(category='vr_roc', id=self.agent.agent_id, msg='   :   | c) Pub Edge Name')
        if self.use_topic_publishing:
            self.closest_node_pub.publish(String(data=edge.split('_')[percentage >= 0.5]))
            self.current_node_pub.publish(String())
            self.closest_edges_pub.publish(ClosestEdges(edge_ids=[edge], distances=[0.0]))
            self.current_edge_pub.publish(CurrentEdge(edge_id=edge))
        else:
            self.agent.location.closest_node_cb(String(data=edge.split('_')[percentage >= 0.5]))
            self.agent.location.current_node_cb(String())
            self.agent.location.closest_edges_cb(ClosestEdges(edge_ids=[edge], distances=[0.0]))
            self.agent.location.current_edge_cb(CurrentEdge(edge_id=edge))

    def publish_path(self, route):
        #logmsg(category='vr_roc', id=self.agent.agent_id, msg='   :   | d) Pub Path')
        path = Path()
        path.header.frame_id = "map"
        if route.source:
            path.poses = [self.get_pose(self.agent.location())]
            path.poses += [self.get_pose(node) for node in route.source]
        self.rviz_route_publisher.publish(path)


    """ --------------- UTILS 3 ------------- """

    def path_gen(self, route, format='   | %s', colour_head=False):
        if not route.source and not route.edge_id: return

        path = []

        # Identify starting list
        if route.source and not route.edge_id:
            # A
            # None
            # + --
            # A
            list_1, list_2 = route.source, []

        elif route.edge_id and not route.source:
            # None
            # AB
            # + --
            # AB
            list_1, list_2 = route.edge_id, []

        elif route.edge_id[0].startswith(route.source[0]):
            # A      B
            #    AB     BC
            # + -----------
            # A  AB  B  BC
            list_1, list_2 = route.source, route.edge_id
        else:
            #     B      C
            # AB     BC
            # + -----------
            # AB  B  BC  C
            list_1, list_2 = route.edge_id, route.source

        # List 1 should be longer than List 2
        if len(list_2) > len(list_1):
            print("we have a problem here...")

        # Loop through all of smaller lists elements:
        for i in range(len(list_2)):
            path += [list_1[i], list_2[i]]

        # Add remaining item
        path += list_1[len(list_2):]
        [logmsg(level='info', category='vr_rod', id=self.agent.agent_id, msg=format%i) for i in path]




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

