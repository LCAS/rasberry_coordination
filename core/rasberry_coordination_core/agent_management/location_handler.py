# Builtins
from copy import deepcopy
import yaml

# Messages
from std_msgs.msg import Bool, String as Str, Empty
from topological_navigation_msgs.msg import ClosestEdges, CurrentEdge
from rasberry_coordination_msgs.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption

# ROS2
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup as RCG
from rasberry_coordination_core.node import GlobalNode

# Logging
from rasberry_coordination_core.utils.logmsg import logmsg

class LocationObj(object):

    def __init__(self, agent, has_presence = True, initial_location = None):
        self.agent = agent
        self.has_presence = has_presence

        self.current_node = initial_location
        self.closest_node = None
        self.previous_node = None

        self.current_edge = None
        self.closest_edge = None

    def enable_location_monitoring(self, agent_id):
        # callback are enabled in base.StageDef.WaitForLocalisation._start()
        ns = f'/{agent_id}/'
        self.current_node_sub = GlobalNode.create_subscription(Str, f'{ns}current_node', self.current_node_cb, 10, callback_group=RCG())
        self.closest_node_sub = GlobalNode.create_subscription(Str, f'{ns}closest_node', self.closest_node_cb, 10, callback_group=RCG())
        self.current_edge_sub = GlobalNode.create_subscription(CurrentEdge, f'{ns}current_edge', self.current_edge_cb, 10, callback_group=RCG())
        self.closest_edges_sub = GlobalNode.create_subscription(ClosestEdges, f'{ns}closest_edges', self.closest_edges_cb, 10, callback_group=RCG())
        self.disable_loc = GlobalNode.create_subscription(Str, f'{ns}localisation/disable', self.disable_localisation, 10, callback_group=RCG())
        self.enable_loc  = GlobalNode.create_subscription(Empty, f'{ns}localisation/enable', self.enable_localisation, 10, callback_group=RCG())

    def __call__(self, accurate=False):
        if accurate:
            return self.current_node or self.previous_node or self.closest_node
        return self.current_node or self.closest_node or self.previous_node

    def current_node_cb(self, msg):
        self.previous_node = self.current_node if self.current_node else self.previous_node
        self.current_node = None if msg.data == "none" else msg.data

    def closest_node_cb(self, msg):
        self.closest_node = None if msg.data == "none" else msg.data

    def current_edge_cb(self, msg):
        self.current_edge = msg.edge_id

    def closest_edges_cb(self, msg):
        self.closest_edge = msg.edge_ids[msg.distances.index(min(msg.distances))] if msg.distances else "none"

    def disable_localisation(self, msg):
        if True: #self.agent.map.is_node(msg): msg in self.agent.empty_node_list!??!?!
            self.current_node_sub = None
            self.closest_node_sub = None
            self.current_node_cb(msg)
            self.closest_node_cb(msg)
        else:
            logmsg(level='warn', agent=self.agent.agent_id, msg="canot fake localisation to node: %s" % str(msg))

    def enable_localisation(self, msg):
        self.previous_node, self.current_node, self.closest_node = None, None, None
        global Subscriber
        self.current_node_sub = Subscriber(self.picker_id + "/current_node", Str, self.current_node_cb)
        self.closest_node_sub = Subscriber(self.picker_id + "/closest_node", Str, self.closest_node_cb)
