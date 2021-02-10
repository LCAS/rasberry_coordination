#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------


""" Agent Details """
class AgentManager(object):
    __metaclass__ = ABCMeta  # @abstractmethod

    """ Initialisation """
    def __init__(self, callback_dict):
        self.agent_details = {}
        self.cb = callback_dict
    def add_agents(self, agent_id_list):
        for agent_id in agent_id_list:
            self.add_agent(agent_id)
    @abstractmethod
    def add_agent(self, agent_id):
        pass

    """ Conveniences """
    def __getitem__(self, key):
        return self.agent_details[key] if key in self.agent_details else None

class RobotManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent_id):
        self.agent_details[agent_id] = RobotDetails(agent_id, self.cb)
class PickerManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent_id):
        self.agent_details[agent_id] = PickerDetails(agent_id, self.cb)
class StorageManager(AgentManager):
    """ Initialisation """
    def add_agent(self, agent_id):
        self.agent_details[agent_id] = StorageDetails(agent_id, self.cb)


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

    """ Initialisation """
    @abstractmethod
    def __init__(self, ID):
        self.agent_id = ID
        self.idle_task_definition = None #TaskDef.idle_picker
        self.new_task_definition = {'default': None} #TaskDef.transportation_courier
        self.interface = None
        pass
    def start_idle_task(self):
        self.idle_task_definition(self)
    def start_new_task(self, task='default', details={}):
        self.new_task_definition[task](self, details)

    """ Localisation """
    def current_node(self, msg):
        self.previous_node = self.current_node if self.current_node else self.previous_node
        self.current_node = None if msg.data == "none" else msg.data
        if self.cb['update_topo_map']:
            self.cb['update_topo_map']()
    def closest_node(self, msg):
        self.closest_node = None if msg.data == "none" else msg.data
    def location(self):
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

class StorageDetails(AgentDetails):
    def __init__(self):
        self.idle_task_definition = TaskDef.idle_courier
        self.new_task_definition = TaskDef.transportation_courier
        self.interface = Robot_Interface(agent_id=self.agent_id,
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
class PickerDetails(AgentDetails):
    def __init__(self):
        self.idle_task_definition = TaskDef.idle_picker
        self.new_task_definition = TaskDef.transportation_picker
        self.interface = LAR_Device(agent_id=self.agent_id,
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
class CourierDetails(AgentDetails):
    def __init__(self, ID):
        self.idle_task_definition = TaskDef.idle_courier
        self.active_task_definition = {'default': TaskDef.transportation_courier}
        self.interface = Robot_Interface(agent_id=self.agent_id,
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


""" Agent Interface Devices / Systems """
class AgentInterface(object):
    def __init__(self, agent_id, responses, sub_topic, pub_topic):
        self.sub = rospy.Subscriber(sub_topic, Str, self.callback)
        self.pub = rospy.Publisher(pub_topic, Str)
        self.agent_id = agent_id
        self.responses = responses
    def callback(self, msg): #Look into sub/feature
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

