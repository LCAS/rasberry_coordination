#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

# Builtins
import traceback
import time, datetime
import yaml

# Messages
from std_msgs.msg import Empty, String
import topological_navigation_msgs.msg
from rasberry_coordination_msgs.msg import Configuration

# ROS2
from rasberry_coordination_core.node import GlobalNode
from rclpy.qos import QoSProfile, DurabilityPolicy

# Components
from rasberry_coordination_core.routing_management.fragment_planner import FragmentPlanner

# Logging
from rasberry_coordination_core.utils.logmsg import logmsg


class RoutingManager(object):
    def __init__(self, agent_manager, planning_format):
        """ Class to create a route_planner object and manage finding of routes (in both empty and congested maps)

        :param planning_type: the planning framework to use (eg. str:"fragment_planner")
        :param robots: pointer to robot_manager
        :param pickers: pointer to picker_manager
        :param callbacks: list of callbacks for direct access to functionality within the coordinator
        """

        # Setup Route Management Tools
        self.trigger_fresh_replan = False #ReplanTrigger
        self.last_replan_time = time.time()
        self.force_replan_to_publish = False
        self.log_routes = True
        global Subscriber
        self.force_replan_cb = GlobalNode.create_subscription(Empty, '~/routing_management/force_replan', self.force_replan, 0)

        # Define route planner properties
        self.planning_type = planning_format['planning_type']
        self.heterogeneous_map = planning_format['heterogeneous_map']
        self.agent_manager = agent_manager

        # Construct the route planner
        topic = '~/routing_management/change_route_planner'
        self.planner = None
        self.planner_id = self.planning_type
        self.change_planner(String(data=self.planning_type))
        self.change_planner_sub = GlobalNode.create_subscription(String, topic, self.change_planner, 0)

        # Initialise the broadcaster
        topic = '~/routing_management/configuration'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.active_planner_pub = GlobalNode.create_publisher(Configuration, topic, qos)


    def change_planner(self, msg): self.planner_id = msg.data
    def create_planner(self):
        # Construct the route planner
        planning_types = {'fragment_planner': self.fragment_planner,
                          'alternative_planner': self.alternative_planner}
        self.planner = planning_types[self.planner_id]()

        # Publicise the currently used planner
        conf = Configuration()
        conf.map='riseholme' # TODO: make this self.planner.map['meta']['pointset']
        conf.route_planner=str(self.planner_id)
        self.active_planner_pub.publish(conf)

        # Clear trigger
        self.planner_id = None

    def find_routes(self):
        """ Proxy function to self.planner.find_routes()

        :return: None
        """
        try:
            self.planner.find_routes()
            self.last_replan_time = time.time()
        except AttributeError as e:
            print(traceback.format_exc())
            logmsg(level="error", category="route", msg='find_routes encountered a problem')

    def fragment_planner(self):  # TODO: add direct object creation in __init__
        """ Create a FragmentPlanner object and populate it with the pointers to the agent managers and the callbacks

        :return: FragmentPlanner
        """
        return FragmentPlanner(self.agent_manager, self.heterogeneous_map)

    def alternative_planner(self):
        """ Example function to show how planning_types dict can be expanded in __init__
        Remove once second planner is added.

        :return: None
        """
        print('placeholder in ' + str(self.__class__))
        # return alternative_planner(self.agent_manager, self.heterogeneous_map)


    """ Publish route if different from current """
    def publish_routes(self, agent, trigger=False):
        logmsg(category="navig", id=agent.agent_id, msg="Attempting to publish route.")

        """ Publish ExecutePolicyModeGoal if different from current policy """
        policy = topological_navigation_msgs.msg.ExecutePolicyModeGoal()

        """ Define route, if no new route is generated, dont do anything. """
        policy.route.source = agent.route_fragments[0] if agent.route_fragments else None
        policy.route.edge_id = agent.route_edges[0] if agent.route_edges else None

        """ Flag to identify if new route is the same and should not be re-published """
        publish_route = True #assume route is identical
        rationalle_to_publish = ""
        reason_failed_to_publish = ""

        """ Identify key elements in routes. """
        old_goal = agent.modules['navigation'].interface.execpolicy_goal
        old_node = old_goal.route.source if old_goal else []
        old_edge = old_goal.route.edge_id if old_goal else []
        new_node = policy.route.source
        new_edge = policy.route.edge_id

        """ If no new route is generated, dont do anything. """
        if (not new_node) or (not new_edge): return

        """ If old route exists, check against it """
        if old_node:
            publish_route = False  #assume new route is the same, so no need to publish
            reason_failed_to_publish = "Routes are same."

            """ Do lists have different lengths? """
            # old: R========T
            # new:     R====T
            if len(new_edge) > len(old_edge):
                publish_route = True
                rationalle_to_publish = "New route is larger then old route."
                logmsg(category="navig", msg="   | new route longer than existing one")
            else:
                publish_route = False
                reason_failed_to_publish = "New shorter route could just be a partially used route"


            """ Compare routes for any differences from target to start. """
            if not publish_route:
                N = new_edge[::-1] #reverse
                O = old_edge[::-1] #reverse
                paired = zip(O[0:len(N)], N)  #trim old route

                for o, n in paired:
                    if o != n:
                        publish_route = True
                        rationalle_to_publish = "New route takes a different route to target."
                        logmsg(category="navig", msg="   | new route different from existing route")
                        break
                    else:
                        publish_route = False
                        reason_failed_to_publish = "Old route uses same path as new route."

        """ If publish_route is True, routes are different """
        if publish_route or self.force_replan_to_publish:
            self.force_replan_to_publish = False
            if self.log_routes:
                new_policy = policy.route.source
                if policy.route.edge_id: new_policy += [policy.route.edge_id[-1].split('_')[1]]
                logmsg(category="navig",  msg="   | New Route:")
                [logmsg(category="navig", msg="   :   | %s" % n) for n in new_policy]
                if not new_policy:
                    logmsg(category="navig", msg="   :   | (empty)")

                oldy = agent.modules['navigation'].interface.execpolicy_goal
                logmsg(category="navig",  msg="   | Prior Route:")
                if not oldy:
                    logmsg(category="navig",  msg="   :   | (empty)")
                else:
                    old_policy = oldy.route.source
                    if oldy.route.edge_id: old_policy += [oldy.route.edge_id[-1].split('_')[1]]
                    [logmsg(category="navig", msg="   :   | %s" % n) for n in old_policy]
                    if not old_policy:
                        logmsg(category="navig", msg="   :   | (empty)")


            agent.modules['navigation'].interface.cancel_execpolicy_goal()
            agent.modules['navigation'].interface.set_execpolicy_goal(policy)
            if 'rasberry_health_monitoring_pkg' in agent.modules:
                if agent.modules['rasberry_health_monitoring_pkg'].interface.is_navigation_available():
                    agent.speaker("caution: moving")
                else:
                    agent.modules['rasberry_health_monitoring_pkg'].interface.say_navigation_block()

            # Route has now been published
            agent().route_required = False
            logmsg(category="navig", id=agent.agent_id, msg="   | route published: %s" % rationalle_to_publish)

            now = str(datetime.datetime.utcnow())
            path = "~/routing/filtered_map_%s.prof" % (now.replace(' ','-'))
            with open("output_file.txt", "w") as f_handle:
                yaml.dump(policy, f_handle)

        else:
            logmsg(category="navig", id=agent.agent_id, msg="   | route failed to published: %s" % reason_failed_to_publish)

        agent().route_found = False  # Route has now been published


    def force_replan(self, msg=None):
        logmsg(category="route", id="PLANNER", msg="Replanning [forced]")
        self.trigger_fresh_replan = True
        self.force_replan_to_publish = True

    def trigger_replan(self):
        logmsg(category="route", id="PLANNER", msg="Replanning [trigger]")
        self.trigger_fresh_replan = True

    def trigger_routing(self, A):
        if self.trigger_fresh_replan:
            self.trigger_fresh_replan = False

        elif any([a().route_required for a in A]):
            logmsg(category="route", id="PLANNER", msg="Replanning [route requires]")

        elif any([a for a in A if a.goal()]) and (time.time() - self.last_replan_time) > 100:
            logmsg(category="route", id="PLANNER", msg="Replanning [timeout]")
            self.last_replan_time = time.time()

        else:
            return False

        return True
