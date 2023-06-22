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
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped

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
        self.tf_broadcaster = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(GlobalNode)

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


############################################################################################

    def cangoal(self):
        """ Remove information on active goal and any buffered goals """
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='Execpolicy Goal Cancelled')
        self.execpolicy_goal, self.buffer = None, []

    def setgoal(self, goal):
        """ Add a new goal to the buffer......................................................................... """
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


############################################################################################

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

        goal = self.execpolicy_goal

        logmsg(category='vr_rod', id=self.agent.agent_id, msg='   | Remaining Route:')
        self.path_gen(goal.route, format='   :   | %s')

        # Move to next node in route
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


############################################################################################


    def wait(self, percentage=1/2):
        #if self.details['smart_delay']: delay = get_smart_travel_time(route.edge) / 2
        delay = percentage * float(fetch_property('navigation', 'debug_robot_step_delay', 2).double_value)
        logmsg(category='vr_rob', id=self.agent.agent_id, msg='   | Travel time: %s seconds' % round(delay,3))
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
            logmsg(level='error', category='vr_roc', id=aid, msg='   :   | Edge not in route, has route changed?')
            return
        self.execpolicy_goal.route.edge_id.pop(0)

    def filter_node(self, node):
        logmsg(category='vr_roc', id=self.agent.agent_id, msg='   | Route Node Popped')
        if not self.execpolicy_goal or self.execpolicy_goal.route.source[0] != node:
            aid = self.agent.agent_id
            logmsg(level='error', category='vr_roc', id=aid, msg='   :   | Node not in route, has route changed?')
            return
        self.execpolicy_goal.route.source.pop(0)


############################################################################################

    def publish_node_pose(self, node):
        """ Publish the robot_pose (& tf) as the pose of the given node """
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
        self.publish_tf(pos, ori)

    def publish_edge_pose(self, edge, percentage=2/3):
        """ Publish the robot_pose (& tf) as pose a percentage through the given edge """
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
        self.publish_tf(pos, ori)

    def publish_tf(self, pos, ori):
        """ Publish the transform from map to robot/base_link given pose info """
        t = TransformStamped()
        t.header.stamp = GlobalNode.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = f"{self.agent.agent_id}/base_link"
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(ori[0])
        t.transform.rotation.y = float(ori[1])
        t.transform.rotation.z = float(ori[2])
        t.transform.rotation.w = float(ori[3])
        self.tf_broadcaster.sendTransform(t)

    def publish_node_name(self, node):
        """ Publish updates to the topological location of the robot when at a node """
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
        """ Publish updates to the topological location of the robot when on an edge """
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
        ''' Publish the path of the robots topological route '''
        path = Path()
        path.header.frame_id = "map"
        if route.source:
            path.poses = [self.get_pose(self.agent.location())]
            path.poses += [self.get_pose(node) for node in route.source]
        self.rviz_route_publisher.publish(path)


############################################################################################


    def path_gen(self, route, format='   | %s', colour_head=False):
        if not route.source and not route.edge_id: return

        path = []

        # Identify starting list
        if route.source and not route.edge_id:
            # Has node, no edges
            # | A
            # | None
            # | + --
            # | A
            list_1, list_2 = route.source, []

        elif route.edge_id and not route.source:
            # Has edges, no node
            # | None
            # | AB
            # | + --
            # | AB
            list_1, list_2 = route.edge_id, []

        elif route.edge_id[0].startswith(route.source[0]):
            # First node proceeds first edge
            # | A      B
            # |    AB     BC
            # | + -----------
            # | A  AB  B  BC
            list_1, list_2 = route.source, route.edge_id
        else:
            # First edge proceeds first node
            # |     B      C
            # | AB     BC
            # | + -----------
            # | AB  B  BC  C
            list_1, list_2 = route.edge_id, route.source

        # Length of List 1 should be greater
        if len(list_2) > len(list_1):
            print("we have a problem here...")

        # Loop through all of smaller lists elements:
        for i in range(len(list_2)):
            path += [list_1[i], list_2[i]]

        # Add remaining item
        path += list_1[len(list_2):]
        [logmsg(category='vr_rod', id=self.agent.agent_id, msg=format%i) for i in path]

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
