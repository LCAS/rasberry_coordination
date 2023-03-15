# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination_core.task_management.modules.base.interfaces.Interface import iFACE as Interface
from rasberry_coordination_core.task_management.modules.navigation.interfaces.GeneralNavigator import iFACE as GeneralNavigator
from rasberry_coordination_core.task_management.__init__ import Stages
from rasberry_coordination_core.task_management.containers.Task import TaskObj as Task
from rasberry_coordination_core.logmsg_utils import logmsg

from random import random
from rclpy.action import ActionClient
import tf2_ros

from std_msgs.msg import Header, String
from nav_msgs.msg import Path
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseStamped, Pose
from strands_navigation_msgs.action import ExecutePolicyMode
from strands_navigation_msgs.msg import TopologicalMap, TopologicalRoute

# Automanaged by rasberry_coordination_core.task_management.__init__.load_modules
# Interface class must be named `iFACE` to be recognised for import
# It will then be identifiable by its Interfaces[<<module>>][<<filename>>]
class iFACE(Interface):
    def __init__(self, agent, details):
        super(iFACE, self).__init__(agent, details)
        self.robot_id = self.agent.agent_id
        self.ns = "/%s/" %(self.robot_id)
        self.speaker_fn = self.agent.speaker

        self.goal_node = "none"
        self.start_time = time.time()
        self.toponav_goal = GotoNodeGoal()
        self.toponav_result = None
        self.toponav_status = None
        self.toponav_route = None

        self.execpolicy_goal = ExecutePolicyModeGoal()
        self.execpolicy_result = None
        self.execpolicy_status = None
        self.execpolicy_current_wp = None

        global Subscriber
        global Publisher

        self.route_search = None
        self.route_publisher = Publisher("/%s/current_route" %(self.robot_id), Path, latch=True, queue_size=5)
        self.topo_route_sub = Subscriber("/%s/topological_navigation/route" %(self.robot_id),
                                          TopologicalRoute, self.topo_route_cb, queue_size=5)

        # navigation action clients
        global ActionClient
        self._exec_policy = ActionClient(ExecutePolicyModeAction, f"{self.ns}topological_navigation/execute_policy_mode")

    def set_execpolicy_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """send_goal to execute_policy_mode action client
        """
        logmsg(category="rob_py", id=self.robot_id, msg='execpolicy goal set')

        if done_cb is None:
            done_cb = self._done_execpolicy_cb
        if feedback_cb is None:
            feedback_cb = self._fb_execpolicy_cb

        self.publish_route(goal.route.source, goal.route.edge_id)

        if len(goal.route.source) > len(goal.route.edge_id):
            del goal.route.source[-1]
        self.execpolicy_goal = goal
        self.execpolicy_current_wp = None
        self.execpolicy_result = None
        self.execpolicy_status = None
        self._exec_policy.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

    def _fb_execpolicy_cb(self, fb):
        """feedback callback
        """
        logmsg(category="rob_py", msg='_fb_execpolicy_cb {current_wp:%s}' % (str(fb.current_wp)))
        self.execpolicy_current_wp = fb.current_wp
        self.execpolicy_status = fb.status

    def _done_execpolicy_cb(self, status, result):
        """done callback
        """
        logmsg(category='rob_py', id=self.robot_id, msg='_done_execpolicy_cb, route has been completed with result: {%s}'%(result))
        if result and result.success == False:
            pass
            #self.speaker_fn("routing cancelled")
        self.execpolicy_status = status
        self.execpolicy_result = result
        self.execpolicy_goal = ExecutePolicyModeGoal()
        self.publish_route()

    def cancel_execpolicy_goal(self, ):
        """
        """
        self._exec_policy.cancel_all_goals()
        self.execpolicy_goal = ExecutePolicyModeGoal()
        self.execpolicy_current_wp = None
        self.execpolicy_result = None
        self.execpolicy_status = None
        self.publish_route()

    def publish_route(self, source=None, edge_id=None):
        """publish route that can be visualised in rviz
        """
        source = source if source else []
        edge_id = edge_id if edge_id else []

        route = Path()
        route.header.frame_id = "map"
        if source:
            for node in source:
                # if there is any elements in the route
                node_obj = self.get_node(node)
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = node_obj.pose.position.x
                pose_stamped.pose.position.y = node_obj.pose.position.y
                route.poses.append(pose_stamped)

            # add the goal node, by looking for the edge in the last source node
            for edge in node_obj.edges:
                if edge.edge_id == edge_id[-1]:
                    node_obj = self.get_node(edge.node)
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = "map"
                    pose_stamped.pose.position.x = node_obj.pose.position.x
                    pose_stamped.pose.position.y = node_obj.pose.position.y
                    route.poses.append(pose_stamped)
                    break

        self.route_publisher.publish(route)
        time.sleep(0.1)

