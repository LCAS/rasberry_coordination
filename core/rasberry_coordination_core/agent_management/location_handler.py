# Builtins
from copy import deepcopy
import yaml

# Messages
from std_msgs.msg import Bool, String as Str, Empty
from topological_navigation_msgs.msg import ClosestEdges
from rasberry_coordination_msgs.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption

# ROS2
from rasberry_coordination_core.node import GlobalNode

# Logging
from rasberry_coordination_core.utils.logmsg import logmsg

class LocationObj(object):

    def __init__(self, agent, has_presence = True, initial_location = None):
        self.agent = agent
        self.has_presence = has_presence
        self.current_node = initial_location
        self.previous_node = None
        self.closest_node = None
        self.closest_edge = None

    def enable_location_monitoring(self, agent_id):
        # callback are enabled in base.StageDef.WaitForLocalisation._start()
        self.current_node_sub = GlobalNode.create_subscription(Str, '/%s/current_node'    % agent_id, self.current_node_cb, 10)
        self.closest_node_sub = GlobalNode.create_subscription(Str, '/%s/closest_node'    % agent_id, self.closest_node_cb, 10)
        self.closest_node_sub = GlobalNode.create_subscription(ClosestEdges, '/%s/closest_edges'   % agent_id, self.closest_edges_cb, 10)
        self.disable_loc = GlobalNode.create_subscription(Str, '/%s/localisation/disable' % agent_id, self.disable_localisation, 10)
        self.enable_loc  = GlobalNode.create_subscription(Empty, '/%s/localisation/enable'  % agent_id, self.enable_localisation, 10)

    def __call__(self, accurate=False):
        if accurate:
            return self.current_node or self.previous_node or self.closest_node
        return self.current_node or self.closest_node or self.previous_node

    def current_node_cb(self, msg):
        self.previous_node = self.current_node if self.current_node else self.previous_node
        self.current_node = None if msg.data == "none" else msg.data

    def closest_node_cb(self, msg):
        self.closest_node = None if msg.data == "none" else msg.data

    def closest_edges_cb(self, msg):
        self.closest_edge = msg.edge_ids[msg.distances.index(min(msg.distances))]
        #print(self.closest_edge)

    def disable_localisation(self, msg):
        if True: #self.agent.map.is_node(msg): msg in self.agent.empty_node_list!??!?!
            self.current_node_sub.unregister()
            self.closest_node_sub.unregister()
            self.current_node_cb(msg)
            self.closest_node_cb(msg)
        else:
            logmsg(level='warn', agent=self.agent.agent_id, msg="canot fake localisation to node: %s" % str(msg))

    def enable_localisation(self, msg):
        self.previous_node, self.current_node, self.closest_node = None, None, None
        global Subscriber
        self.current_node_sub = Subscriber(self.picker_id + "/current_node", Str, self.current_node_cb)
        self.closest_node_sub = Subscriber(self.picker_id + "/closest_node", Str, self.closest_node_cb)
