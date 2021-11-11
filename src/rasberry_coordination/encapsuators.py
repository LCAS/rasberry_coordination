from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy

from std_msgs.msg import Bool, String as Str, Empty as Emp
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
        
    def enable_location_monitoring(self, agent_id):
        # callback are enabled in base.StageDef.WaitForLocalisation._start()
        self.current_node_sub = Subscriber('/%s/current_node'    % agent_id, Str, self.current_node_cb)
        self.closest_node_sub = Subscriber('/%s/closest_node'    % agent_id, Str, self.closest_node_cb)
        self.disable_loc = Subscriber('/%s/localisation/disable' % agent_id, Str, self.disable_localisation)
        self.enable_loc  = Subscriber('/%s/localisation/enable'  % agent_id, Emp, self.enable_localisation)

    def __call__(self, accurate=False):
        if accurate:
            return self.current_node or self.previous_node or self.closest_node
        return self.current_node or self.closest_node or self.previous_node

    def current_node_cb(self, msg):
        self.previous_node = self.current_node if self.current_node else self.previous_node
        self.current_node = None if msg.data == "none" else msg.data

    def closest_node_cb(self, msg):
        self.closest_node = None if msg.data == "none" else msg.data

    def disable_localisation(self, msg):
        self.current_node_sub.unregister()
        self.closest_node_sub.unregister()
        self.current_node_cb(msg.data)
        self.closest_node_cb(msg.data)

    def enable_localisation(self, msg):
        self.previous_node, self.current_node, self.closest_node = None, None, None
        self.current_node_sub = Sub(self.picker_id + "/current_node", Str, self.current_node_cb)
        self.closest_node_sub = Sub(self.picker_id + "/closest_node", Str, self.closest_node_cb)


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
        logmsg(category="module", msg="%s (%s)"%(name.upper(),role.upper()))
        self.agent = agent
        self.name = name
        self.role = role

        interface_name = '%s_%s' % (name, role)
        from rasberry_coordination.task_management.__init__ import InterfaceDef, PropertiesDef
        definition = getattr(InterfaceDef, interface_name)

        self.interface = definition(agent=agent)
        self.properties = PropertiesDef[name] if name in PropertiesDef else dict()

        self.init_task_name = '%s_init' % (interface_name)
        self.idle_task_name = '%s_idle' % (interface_name)

        self.add_init_task()

    def add_init_task(self):
        self.agent.add_task(task_name=self.init_task_name)

    # def add_idle_task(self):
    #     if getattr(TaskDef, self.idle_task_name):
    #         self.agent.add_task(task_name=self.idle_task_name)









