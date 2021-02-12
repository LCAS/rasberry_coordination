#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------


from abc import ABCMeta, abstractmethod
from rospy import Subscriber, Publisher
from std_msgs.msg import String as Str
from rasberry_coordination.msg import KeyValuePair
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.task_management.Tasks import TaskDef

""" Agent Details """
class AgentManager(object):
    __metaclass__ = ABCMeta  # @abstractmethod

    """ Initialisation """
    def __init__(self, callback_dict):
        self.agent_details = {}
        self.cb = callback_dict
    def add_agents(self, agent_list):
        for agent in agent_list:
            self.add_agent(agent)
    @abstractmethod
    def add_agent(self, agent):
        pass

    """ Conveniences """
    def __getitem__(self, key):
        return self.agent_details[key] if key in self.agent_details else None

class CourierManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent):
        self.agent_details[agent['agent_id']] = CourierDetails(agent, self.cb)
class PickerManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent):
        self.agent_details[agent['agent_id']] = PickerDetails(agent, self.cb)
class StorageManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent):
        self.agent_details[agent['agent_id']] = StorageDetails(agent, self.cb)


""" Agent Management """
class AgentDetails(object):
    __metaclass__ = ABCMeta #@abstractmethod
    """ Fields:
    - agent_id, agent_type
    - idle_task_definition, new_task_definition
    - interface
    - task_stage_list, task_details
    - (subs) current_node, closest_node, previous_node
    """

    """ Initialisations """
    @abstractmethod
    def __init__(self, agent, callbacks):
        self.agent_id = agent['agent_id']
        self.cb = callbacks

        #Task Defaults
        self.task_stage_list = []
        self.idle_task_default = agent['idle_task_default']
        self.new_task_default = agent['new_task_default']

        #Location and Callbacks
        self.subs = {}
        self.current_node = None
        self.previous_node = None
        self.closest_node = None
        if 'initial_location' in agent:
            self.current_node = agent['initial_location']
        self.subs['current_node'] = Subscriber('/%s/current_node'%(self.agent_id), Str, self.current_node_cb)
        self.subs['closest_node'] = Subscriber('/%s/closest_node'%(self.agent_id), Str, self.closest_node_cb)

    def start_idle_task(self, task="default"):
        task = task if task in self.idle_task_definition else self.new_task_default
        self.idle_task_definition[task](self)
    def start_new_task(self, task, details={}):
        task = task if task in self.new_task_definition else self.new_task_default
        self.new_task_definition[task](self, details)

    """ Localisation """
    def current_node_cb(self, msg):
        self.previous_node = self.current_node if self.current_node else self.previous_node
        self.current_node = None if msg.data == "none" else msg.data
        if self.cb['update_topo_map']:
            self.cb['update_topo_map']()
    def closest_node_cb(self, msg):
        self.closest_node = None if msg.data == "none" else msg.data
    def location(self, accurate=False):
        if accurate:
            return self.current_node or self.previous_node
        return self.current_node or self.closest_node or self.previous_node

    """ Conveniences """
    def __call__(A, index=0):
        return A.task_stage_list[index]
    def __getitem__(A, key):
        return A.task_details[key] if key in A.task_details else None
    def __setitem__(A, key, val):
        A.task_details[key] = val

    """ Standard Task Interactions """
    def notify(self, state):
        self.interface.publish(state)
    def flag(self, flag):
        self['stage_complete_flag'] = flag
    def end_stage(self):
        self.task_stage_list.pop(0)

    """ Logging """
    def __repr__(self):
        return self.get_class()
    def get_class(self):
        return str(self.__class__).replace("<class 'rasberry_coordination.agent_managers.store_manager.", "").replace("'>", "")


class CourierDetails(AgentDetails):
    def __init__(self, agent, callbacks):
        super(CourierDetails, self).__init__(agent, callbacks)
        self.idle_task_definition = {'default': TaskDef.idle_courier}
        self.active_task_definition = {'default': TaskDef.transportation_courier,
                                       'low_battery': TaskDef.charge_robot} #i.e.
        interfaces = {"robot_interface":Robot_Interface}
        self.interface = interfaces[agent['interface_type']](agent_id=self.agent_id,
                                                             responses={'PAUSE'  :self.pause,
                                                                        'UNPAUSE':self.unpause,
                                                                        'RELEASE':self.release})
        pass
    def pause(self):
        self.task_stage_list.insert(0, StageDef.Pause(self))
    def unpause(self):
        self.registration = True
    def release(self):
        self.task_stage_list = []
class PickerDetails(AgentDetails):
    def __init__(self, agent, callbacks):
        super(PickerDetails, self).__init__(agent, callbacks)
        self.idle_task_definition = {'default': TaskDef.idle_picker}
        self.new_task_definition = {'default': TaskDef.transportation_request}
        interfaces = {"car_app":CAR_App,
                      "lar_app":LAR_App,
                      "car_device":CAR_Device,
                      "lar_device":LAR_Device}
        self.interface = interfaces[agent['interface_type']](agent_id=self.agent_id,
                                                             responses={'CALLED':self.called,
                                                                        'LOADED':self.loaded,
                                                                        'INIT'  :self.reset})
        pass
    def called(self):
        self['start_time'] = Now()  #self['task_id'] = task_id_made_by_picker_manager
        self['task_id'] = "%s_%s"%(self.agent_id,self.total_tasks)
        self.total_tasks += 1
    def loaded(self):
        self['picker_has_tray'] = False
    def reset(self):
        pass
class StorageDetails(AgentDetails):
    def __init__(self, agent, callbacks):
        super(StorageDetails, self).__init__(agent, callbacks)
        self.idle_task_definition = {'default': TaskDef.idle_courier}
        self.new_task_definition = {'default': TaskDef.transportation_courier}
        interfaces = {"car_app":CAR_App,
                      "uar_app":UAR_App,
                      "car_device":CAR_Device,
                      "uar_device":UAR_Device}
        self.interface = interfaces[agent['interface_type']](agent_id=self.agent_id,
                                                             responses={'UNLOADED' :self.unloaded,
                                                                        'OFFLINE'  :self.offline,
                                                                        'ONLINE'   :self.online})
        pass
    def unloaded(self):
        self['storage_has_tray'] = True
    def offline(self):
        pass
    def online(self):
        pass


""" Agent Interface Devices / Systems """
class AgentInterface(object):
    def __init__(self, agent_id, responses, sub_topic, pub_topic):
        self.sub = Subscriber(sub_topic, Str, self.callback)
        self.pub = Publisher(pub_topic, Str, queue_size=5)
        self.agent_id = agent_id
        self.responses = responses
    def callback(self, msg): #Look into sub/feature
        msg = eval(msg)
        if msg["user"] == self.agent_id:
            self.responses[msg["state"]]()
    def notify(self, state):
        self.pub.publish(str({'user': self.agent_id, 'state': state}))

#Definitions for CallARobot/LoadARobot (picker), UnloadARobot (storage), and Robot_Interface (robot)
class CAR_App(AgentInterface):
    def __init__(self, agent_id, responses, sub_topic='/car/get_states', pub_topic='/car/set_states'):
        super(CAR_App, self).__init__(agent_id, responses, sub_topic, pub_topic)
class LAR_Device(AgentInterface):
    def __init__(self, agent_id, responses, sub_topic='/lar/get_states', pub_topic='/lar/set_states'):
        super(LAR_Device, self).__init__(agent_id, responses, sub_topic, pub_topic)
class UAR_Device(AgentInterface):
    def __init__(self, agent_id, responses, sub_topic='/uar/get_states', pub_topic='/uar/set_states'):
        super(UAR_Device, self).__init__(agent_id, responses, sub_topic, pub_topic)
class Robot_Interface(AgentInterface):
    def __init__(self, agent_id, responses, sub_topic='/robot/get_states', pub_topic='/robot/set_states'):
        super(Robot_Interface, self).__init__(agent_id, responses, sub_topic, pub_topic)
#TODO \/
class LAR_App(LAR_Device):
    pass
class UAR_App(UAR_Device):
    pass
class CAR_Device(AgentInterface):
    pass
class CAR_App(CAR_Device):
    pass
