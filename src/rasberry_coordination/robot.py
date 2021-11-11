#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import actionlib
import rospy
import yaml
from time import sleep

from std_msgs.msg import String, Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from rasberry_coordination.coordinator_tools import logmsg
from strands_navigation_msgs.msg import NavRoute, ExecutePolicyModeGoal, TopologicalRoute, ExecutePolicyModeAction

from topological_navigation.msg import GotoNodeGoal, GotoNodeAction
from topological_navigation.tmap_utils import get_node_from_tmap2 as GetTopoMapNode
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch


class Robot(object):
    """Robot class to wrap all ros interfaces to the physical/simulated robot
    """
    def __init__(self, robot_id, ):
        """initialise the Robot class

        Keyword arguments:

        robot_id - id of robot
        """
        logmsg(category="rob_py", msg="    | Starting Robot.py:")

        self.robot_id = robot_id
        self.ns = "/%s/" %(robot_id)

        self.goal_node = "none"
        self.start_time = rospy.get_rostime()
        self.toponav_goal = GotoNodeGoal()
        self.toponav_result = None
        self.toponav_status = None
        self.toponav_route = None

        self.execpolicy_goal = ExecutePolicyModeGoal()
        self.execpolicy_result = None
        self.execpolicy_status = None
        self.execpolicy_current_wp = None

        self.topo_map = None
        self.rec_topo_map = False
        self.route_search = None
        self.route_publisher = rospy.Publisher("%s/current_route" %(self.robot_id), Path, latch=True, queue_size=5)

        self.total_fails = 0
        self.force_replan = rospy.Publisher("/rasberry_coordination/force_replan", Empty, queue_size=5)

        rospy.Subscriber("/topological_map_2", String, self._map_cb)
        logmsg(category="rob_py", msg='    |    | awaiting topomap2')
        while not self.rec_topo_map: rospy.sleep(rospy.Duration.from_sec(0.1))
        logmsg(category="rob_py", msg='    |    | received topomap2')

        self.route_search = TopologicalRouteSearch(self.topo_map)
        self.publish_route()

        self.topo_route_sub = rospy.Subscriber("/%s/topological_navigation/Route" %(self.robot_id), TopologicalRoute,
                                               self.topo_route_cb,
                                               queue_size=5)

        # topological navigation action client
        self._topo_nav = actionlib.SimpleActionClient(self.ns + "topological_navigation", GotoNodeAction)

        # execute policy action client
        self._exec_policy = actionlib.SimpleActionClient(self.ns + "topological_navigation/execute_policy_mode", ExecutePolicyModeAction)

    def _map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = yaml.safe_load(msg.data)
        self.rec_topo_map = True
        if not self.route_search:
            self.route_search = TopologicalRouteSearch(self.topo_map)
            self.publish_route()

    def topo_route_cb(self, msg):
        """callback for topological_navigation/Route messages
        """
        edges = []
        for i in range(len(msg.nodes)-1):
            _, _edges = self.get_path(msg.nodes[i], msg.nodes[i+1])
            for edge in _edges:
                edges.append(edge)
        self.publish_route(source=msg.nodes, edge_id=edges)

    def get_node(self, node):
        """get_node: Given a node name return its node object.
        A wrapper for the get_node function in tmap_utils

        Keyword arguments:

        node -- name of the node in topological map"""
        return GetTopoMapNode(self.topo_map, node)

    def get_path(self, start_node, goal_node):
        """get route_nodes and route_edges from start_node to goal_node

        Keyword arguments:

        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        logmsg(category="rob_py", msg='get_path from start_node:[%s] to goal_node:[%s]'%(start_node,goal_node))

        if not self.route_search:
            logmsg(level="error", category="rob_py", msg='self.route_search not initialised')
            return([], [], [float("inf")])

        route = self.route_search.search_route(start_node, goal_node)
        if route in [None, NavRoute()]:
            logmsg(category="rob_py", id=self.robot_id, msg='no route found between %s and %s' %(start_node, goal_node))
            #TODO: Set this up so it doesnt repeat along with with "logwarn(replanning now)" (throttle?)
            return ([], [], [float("inf")])

        route_nodes = route.source
        route_nodes.append(goal_node)
        route_edges = route.edge_id

        return (route_nodes, route_edges)

    def set_toponav_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """send_goal to topo_nav action client
        """
        logmsg(category="rob_py", id=self.robot_id, msg='assigned toponav goal set as %s'%(goal.target))
        if done_cb is None:
            done_cb = self._done_toponav_cb
        if feedback_cb is None:
            feedback_cb = self._fb_toponav_cb

        self.toponav_goal = goal
        self.toponav_result = None
        self.toponav_route = None
        self.toponav_status = None
        logmsg(category="rob_py", msg="Route published: _topo_nav.send_goal")
        self._topo_nav.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
        # self._topo_nav.wait_for_result()

    def _fb_toponav_cb(self, fb):
        """feedback callback
        """
        self.toponav_route = fb.route
        self.publish_route()

    def _done_toponav_cb(self, status, result):
        """done callback
        """
        self.toponav_goal = GotoNodeGoal()
        self.toponav_status = status
        self.toponav_result = result
        self.publish_route()

    def cancel_toponav_goal(self, ):
        """
        """
#        print ("%s cancelling execute policy mode goal" %(self.robot_id))
        self._topo_nav.cancel_all_goals()
        self.toponav_goal = GotoNodeGoal()
        self.toponav_result = None
        self.toponav_route = None
        self.toponav_status = None
        self.publish_route()

    def set_execpolicy_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """send_goal to execute_policy_mode action client
        """
        logmsg(category="rob_py", id=self.robot_id, msg="    - execpolicy goal set")

        if done_cb is None:
            done_cb = self._done_execpolicy_cb
        if feedback_cb is None:
            feedback_cb = self._fb_execpolicy_cb

        self.publish_route(goal.route.source, goal.route.edge_id)

        self.execpolicy_goal = goal
        self.execpolicy_current_wp = None
        self.execpolicy_result = None
        self.execpolicy_status = None
        if goal == ExecutePolicyModeGoal():
            # sending empty route will cause error in the action server. avoid it by cancelling it
            logmsg(category="rob_py", msg="    - empty route in action server cancelled.")
            print(goal)
            logmsg(level="error", category="test", msg="    - empty route action server::cancel_execpolicy_goal")
            self.cancel_execpolicy_goal()
        else:
            logmsg(category="rob_py", msg="    - route published: _exec_policy.send_goal")
            print(goal)
            self._exec_policy.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
#        self._exec_policy.wait_for_result()

    def _fb_execpolicy_cb(self, fb):
        """feedback callback
        """
        logmsg(category="robnav", msg='_fb_execpolicy_cb {current_wp:%s}' % (str(fb.current_wp)))
        self.execpolicy_current_wp = fb.current_wp
        self.execpolicy_status = fb.status

    def _done_execpolicy_cb(self, status, result):
        """done callback
        """
        logmsg(category='rob_py', id=self.robot_id, msg='_done_execpolicy_cb, route completed: {%s}'%(result))

        if result.success == False:
            self.total_fails += 1
            logmsg(level='error', category='rob_py', msg='_done_execpolicy_cb, failed to complete route')
            if self.total_fails > 5:
                logmsg(level='error', category='rob_py', msg='self.total_fails > 5')
            else:
                logmsg(level='error', category='rob_py', msg='publishing to force replan')
                sleep(5)
                self.force_replan.publish(Empty())
        else:
            logmsg(level='error', category='rob_py', msg='route success')
            self.total_fails = 0

        self.execpolicy_status = status
        self.execpolicy_result = result
        self.execpolicy_goal = ExecutePolicyModeGoal()
        self.publish_route()

    def cancel_execpolicy_goal(self, ):
        logmsg(category="test", msg="::cancel_execpolicy_goal")

        self._exec_policy.cancel_all_goals()
        self.execpolicy_goal = ExecutePolicyModeGoal()
        self.execpolicy_current_wp = None
        self.execpolicy_result = None
        self.execpolicy_status = None
        self.publish_route()

    def publish_route(self, source=[], edge_id=[]):
        """publish route that can be visualised in rviz
        """
        route = Path()
        route.header.frame_id = "map"
        if source:
            for node in source:
                # if there is any elements in the route
                node_obj = self.get_node(node)
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = node_obj["node"]["pose"]["position"]["x"]
                pose_stamped.pose.position.y = node_obj["node"]["pose"]["position"]["y"]
                pose_stamped.pose.orientation.w = 1
                route.poses.append(pose_stamped)

            # add the goal node, by looking for the edge in the last source node
            for edge in node_obj["node"]["edges"]:
                if edge["edge_id"] == edge_id[-1]:
                    node_obj = self.get_node(edge["node"])
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = "map"
                    pose_stamped.pose.position.x = node_obj["node"]["pose"]["position"]["x"]
                    pose_stamped.pose.position.y = node_obj["node"]["pose"]["position"]["y"]
                    pose_stamped.pose.orientation.w = 1
                    route.poses.append(pose_stamped)
                    break

        self.route_publisher.publish(route)
        rospy.sleep(0.1)