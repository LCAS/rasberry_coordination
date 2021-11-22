from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy

from std_msgs.msg import Bool, String as Str, Empty as Emp
import strands_executive_msgs.msg

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.srv import AgentNodePair



class LocationObj(object):

    def __init__(self, presence = True, initial_location = None):
        self.has_presence = bool(presence)
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

    def __init__(self, id=None, name=None, module=None, details=None, contacts=None, initiator_id=None, responder_id=None, stage_list=None):
        self.id = str() if not id else id
        self.name = str() if not name else name
        self.module = str() if not name else module
        self.details = dict() if not details else details
        self.contacts = dict() if not contacts else contacts
        self.initiator_id = str() if not initiator_id else initiator_id
        self.responder_id = str() if not responder_id else responder_id
        self.stage_list = list() if not stage_list else stage_list


    def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
    def __setitem__(self, key, val): self.__setattr__(key,val)


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









