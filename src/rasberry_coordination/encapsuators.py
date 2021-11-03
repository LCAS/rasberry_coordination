from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy

from std_msgs.msg import Bool, String as Str
import strands_executive_msgs.msg

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.srv import AgentNodePair



class LocationObj(object):
    def __init__(self, presence = True, initial_location = None):
        self.has_presence = presence
        self.current_node = initial_location
        self.previous_node = None
        self.closest_node = None
    def __call__(self, accurate=False):
        if accurate:
            return self.current_node or self.previous_node or self.closest_node
        return self.current_node or self.closest_node or self.previous_node
    def current_node_cb(self, msg):
        self.previous_node = self.current_node if self.current_node else self.previous_node
        self.current_node = None if msg.data == "none" else msg.data
    def closest_node_cb(self, msg):
        self.closest_node = None if msg.data == "none" else msg.data
    def set_location_srv(self, req):
        r = AgentNodePairResponse()
        r.success = False

        if req.node_id:  # set location to given node
            self.current_node_sub.unregister()
            self.closest_node_sub.unregister()
            msg = Str(req.node_id)
            self.current_node_cb(msg)
            self.closest_node_cb(msg)
            r.success = True ; r.msg = "Node Set"
        else:
            self.previous_node, self.current_node, self.closest_node = None, None, None
            self.current_node_sub = Sub(self.picker_id + "/current_node", Str, self.current_node_cb)
            self.closest_node_sub = Sub(self.picker_id + "/closest_node", Str, self.closest_node_cb)
            r.success = True ; r.msg = "Localisation Resumed"
        return resp


class TaskObj(object):

    def __repr__(self):
        return "Task( id:%s | module:%s | name:%s | init:%s | resp:%s | #stages:%s )" % \
               (self.id, self.module, self.name, self.initiator_id, self.responder_id, len(self.stage_list))

    def __init__(self, default=False, **kwargs):
        if default:
            self.load_defaults()
        else:
            for k, v in kwargs.items():
                self.__setattr__(k, v)

    def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
    def __setitem__(self, key, val): self.__setattr__(key,val)

    def load_defaults(self):
        self.id = None
        self.name = None
        self.module = None

        self.details = dict()
        self.contacts = dict()

        self.initiator_id = None
        self.responder_id = None

        self.stage_list = []

        self.action = None

class ModuleObj(object):

    def __repr__(self):
        return "Module( name:%s | role:%s | interface:%s )" % (self.name, self.role, self.interface!=None)

    def __init__(self, agent, name, role):
        self.agent = agent
        self.name = name
        self.role = role

        interface_name = '%s_%s' % (name, role)
        from rasberry_coordination.task_management.__init__ import InterfaceDef, PropertiesDef
        definition = getattr(InterfaceDef, interface_name)

        self.interface = definition(agent=agent)
        self.properties = PropertiesDef[name]

        self.init_task_name = '%s_init' % (interface_name)
        self.idle_task_name = '%s_idle' % (interface_name)

        self.add_init_task()

    def add_init_task(self):
        self.agent.add_task(task_name=self.init_task_name)

    def add_idle_task(self):
        self.agent.add_task(task_name=self.idle_task_name)


# class ActionObj(object):
#     def __init__(self, agent, type, style, descriptor, response_location):
#         self.type = 'find_node'
#         self.style = 'closest'
#         self.descriptor = 'charging_station'
#         self.response_location = None
#
#         pass
#
#     def respond(self, value):
#         self.response_location = value
#
#
#
#     class AssignChargeNode(SDef.AssignNode):
#         def _start(self):
#             super(StageDef.AssignChargeNode, self)._start()
#             self.action['action_type'] = 'find_node'
#             self.action['action_style'] = 'closest'
#             self.action['descriptor'] = 'charging_station'
#             self.action['response_location'] = None
#         def _end(self):
#             self.agent['contacts']['charging_station'] = self.action['response_location']
#
#
#
#     class AssignChargeNode(SDef.AssignNode):
#         def _start(self):
#             super(StageDef.AssignChargeNode, self)._start()
#             self.agent().action = Action(type='find_closest_node', property='charging_station')
#         def _end(self):
#             self.agent['contacts']['charging_station'] = self.agent().action.response_location
#
#
#
#     class AcceptCourier(SDef.AssignAgent):
#         def _start(self):
#             self.action['action_type'] = 'find_agent_from_list'
#             self.action['action_style'] = 'closest'
#             self.action['list'] = self.agent.request_admittance
#             self.action['response_location'] = None
#         def _end(self):
#             self.agent['contacts']['courier'] = self.action['response_location']










