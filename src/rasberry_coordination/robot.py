#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import actionlib
import rospy
import tf
from rospy import Publisher, Subscriber
from rasberry_coordination.coordinator_tools import logmsg

from topological_navigation.tmap_utils import get_node as get_topomap_node, get_distance_node_pose_from_tmap2 as node_pose_dist
from topological_navigation.route_search import TopologicalRouteSearch

from std_msgs.msg import Header, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from strands_navigation_msgs.msg import ExecutePolicyModeGoal, ExecutePolicyModeAction, TopologicalMap, TopologicalRoute
from topological_navigation.msg import GotoNodeGoal, GotoNodeAction


class Robot(object):
    """Robot class to wrap all ros interfaces to the physical/simulated robot
    """
    def __init__(self, robot_id, ):
        """initialise the Robot class

        Keyword arguments:

        robot_id - id of robot
        """
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
        Subscriber("topological_map", TopologicalMap, self._map_cb)
        logmsg(category="rob_py", id=self.robot_id, msg='waiting for Topological map ...')

        while not self.rec_topo_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))
        logmsg(category="rob_py", id=self.robot_id, msg='received Topological map')

        self.route_search = TopologicalRouteSearch(self.topo_map)
        self.route_publisher = Publisher("%s/current_route" %(self.robot_id), Path, latch=True, queue_size=5)
        self.publish_route()

        self.topo_route_sub = Subscriber("/%s/topological_navigation/Route" %(self.robot_id),
                                               TopologicalRoute,
                                               self.topo_route_cb,
                                               queue_size=5)

        # topological navigation action client
        self._topo_nav = actionlib.SimpleActionClient(self.ns + "topological_navigation", GotoNodeAction)

        # execute policy action client
        self._exec_policy = actionlib.SimpleActionClient(self.ns + "topological_navigation/execute_policy_mode", ExecutePolicyModeAction)

    def _map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = msg
        self.rec_topo_map = True

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
        return get_topomap_node(self.topo_map, node)

    def get_path(self, start_node, goal_node):
        """get route_nodes and route_edges from start_node to goal_node

        Keyword arguments:

        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        logmsg(category="rob_py", msg='get_path from start_node:[%s] to goal_node:[%s]'%(start_node,goal_node))
        route = self.route_search.search_route(start_node, goal_node)
        if route is None:
            logmsg(category="rob_py", id=self.robot_id, msg='no route found between %s and %s' %(start_node, goal_node))
            #TODO: Set this up so it doesnt repeat along with with "logwarn(replanning now)"

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
        self._topo_nav.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

#        self._topo_nav.wait_for_result()

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
        self._topo_nav.cancel_all_goals()
        self.toponav_goal = GotoNodeGoal()
        self.toponav_result = None
        self.toponav_route = None
        self.toponav_status = None
        self.publish_route()

    def set_execpolicy_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """send_goal to execute_policy_mode action client
        """
        logmsg(category="rob_py", id=self.robot_id, msg='execpolicy goal set')

        if done_cb is None:
            done_cb = self._done_execpolicy_cb
        if feedback_cb is None:
            feedback_cb = self._fb_execpolicy_cb

        self.publish_route(goal.route.source, goal.route.edge_id)

        self.execpolicy_goal = goal
        self.execpolicy_current_wp = None
        self.execpolicy_result = None
        self.execpolicy_status = None
        self._exec_policy.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
#        self._exec_policy.wait_for_result()

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
        rospy.sleep(0.1)


class VirtualRobot(object):
    def __init__(self, agent_id, agent, step_delay=0.5):
        self.agent = agent
        self.execpolicy_goal, self.route = ExecutePolicyModeGoal(), Path()

        rospy.set_param('/rasberry_coordination/virtual_robot/route_step_delay', step_delay)

        self.topo_map = None
        self.rec_topo_map = False
        Subscriber("topological_map", TopologicalMap, self._map_cb)

        self.route_search = TopologicalRouteSearch(self.topo_map)
        self.route_publisher = Publisher("/%s/current_route" % agent_id, Path, latch=True, queue_size=5)
        self.route_progress_publisher = Publisher("/%s/current_route_progress" % agent_id, Path, latch=True, queue_size=5)

        self.pose_update_pub = Publisher("/%s/new_pose" % agent_id, Pose, queue_size=5)

        self.current_node_pub = Publisher("/%s/current_node" % agent_id, String, latch=True, queue_size=5)
        self.closest_node_pub = Publisher("/%s/closest_node" % agent_id, String, latch=True, queue_size=5)

    def enable_subscribers(self):
        self.route_subscriber = Subscriber("/%s/current_route_progress" % self.agent.agent_id, Path, self.route_sub_cb)
        self.pose_stamped_subscriber = Subscriber("/%s/robot_pose" % self.agent.agent_id, Pose, self.pose_cb)

    def _map_cb(self, msg):
        self.topo_map, self.rec_topo_map = msg, True
    def cancel_execpolicy_goal(self):
        self.execpolicy_goal, self.route = ExecutePolicyModeGoal(), Path()
    def set_execpolicy_goal(self, goal):
        self.publish_route(goal.route.source, goal.route.edge_id)
        self.execpolicy_goal = goal
    def publish_route(self, source=None, edge_id=None):
        """publish route that can be visualised in rviz
        """
        source = source if source else []
        edge_id = edge_id if edge_id else []

        route = Path()
        route.header.frame_id = "map"
        if source:
            route.poses = [self.get_pose(node) for node in source]
            node_obj = self.get_node(source[-1])
            route.poses += [[self.get_pose(edge.node) for edge in node_obj.edges if edge.edge_id == edge_id[-1]][0]]

        self.route_publisher.publish(route)
        self.route_progress_publisher.publish(route)
        self.route = route
    def get_pose(self, node):
        return PoseStamped(header=(Header(frame_id="map")), pose=self.get_node(node).pose)
    def get_node(self, node): return get_topomap_node(self.topo_map, node)

    def route_sub_cb(self, route):
        if route.poses:
            x = route.poses[0].pose
            del route.poses[0]

            self.pose_update_pub.publish(x)  # publish x as new location
            rospy.sleep(rospy.get_param('/rasberry_coordination/virtual_robot/route_step_delay', 2))
            self.route_progress_publisher.publish(route)
        else:
            self.route_publisher.publish(route)


    def pose_cb(self, msg):
        new_node = self.find_closest_node(msg)
        self.current_node_pub.publish(new_node)
        self.closest_node_pub.publish(new_node)

        br = tf.TransformBroadcaster()
        br.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                         (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                         rospy.Time.now(),
                         "%s/base_link" % self.agent.agent_id,
                         "map")

    def find_closest_node(self, pose):
        node_list = self.agent.map_handler.map['nodes']
        dists = {node["node"]["name"]: node_pose_dist(node, pose) for node in node_list}
        return min(dists, key=dists.get)


