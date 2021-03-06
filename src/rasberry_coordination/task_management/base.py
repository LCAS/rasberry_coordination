""" Task Stages:
- All stages must inherit from StageBase.
"""

#Standard Stage Methods:
"""
Standard Stage Methods:
    - name: __init__
      description: Define basic local properties
      super: True

    - name: _start_
      description: 
      super: True

    - name: _query_
      description:

    - name: __del__
      description:
      super: True

    - name: __repr__
      description: 
    - name: _get_class
      description: 
    - name: _summary
      description: 
      
    - name: _notify_start
      description: 
    - name: _notify_end
      description: 
"""

#Standard Stage Attributes:
"""
Standard Stage Attributes:
    - name: agent
      description:
    - name: action_required
      description:
    - name: route_required
      description:
    - name: stage_complete
      description:
    - name: new_stage
      description:
      
    - name: target
      description:
      
    - name: action
      description: Dictionary of details for performing actions
    - name: summary
      description: Dictionary containing descriptiosn of customised functions
      
    - name: start_time
      description: 
      

"""

#Subclass Inheritance Tree
"""
BaseStage.inheritance:
    StartTask:
    TargetedMovement:
        EdgeUVTreatment:
        RowUVTreatment:
    Await:
        AwaitAgentArrival:
        AwaitAgentDismissal:
        AwaitLoading:
        AwaitUnloading:
    Wait:
        WaitDuration:
        Pause:
"""

#
"""
All task specific details must be saved in agent.task_details
Exceptions:
    Non-Task-Based Attributes:
        - location
        - task_details #not directly accessed
    Non-Task-Based Methods:
        - .flag()
        - .new_task()   # 
        - .set_status() #communication
    Values which must persist beyond the end of task
        - store.request_admittance
        

- target   (standard for planning)
- ...
- admittance (requied for communication)

Interference with tasks should not be direct between agents.
- advencement achieved by agents with their own _query
- modify agent.tray_present, not Stage.stage_complete

Methods called within coordinator:
- closest_wait_node()
- closest_storage() #closest_target
- closest_robot()   #closest_target
"""

"""
Rules for Custom Stages:
- all cross-stage details must be stored in Stage.agent.task_details
- all communication must be limited to the Stage._notify* methods
- all overloading of __init__ must include a call to super first
- all overloading of _start must include a call to super
- all overloading of __del__ must include a call to super

Notes:
- __init__ is called when the task stages are added to an agent
- __repr__ can be overloaded to include more information
- _start is called once when the stask is at the head of task_stage_list
- _start can be called a second time by setting Stage.new_stage to True
- _query is called on every iteration of Coordinator.run, except when task_progression is paused
- 

(Tim Peters, The Zen of Python):
"Special cases aren't special enough to break the rules." / "Although practicality beats purity."
-> `python -c "import this"`
"""

from copy import deepcopy
from std_msgs.msg import String as Str
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.coordinator_tools import logmsg

class InterfaceDef(object):
    class AgentInterface(object):
        def __init__(self, agent, responses, sub, pub):
            self.agent = agent
            self.responses = responses
            self.pub = Publisher(pub, Str, queue_size=5)
            self.sub = Subscriber(sub, Str, self.callback, agent.agent_id)
        def callback(self, msg, agent_id):  # Look into sub/feature
            msg = eval(msg.data)
            if "states" in msg: return # car callback sends two msgs, this filters second
            if msg['user'] == agent_id:
                if msg['state'] in self.responses:
                    self.responses[msg['state']]()
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\": \"%s\"}' % (self.agent.agent_id, state))
            logmsg(msg="PUBLISHING \"%s\"" % msg)
            self.pub.publish(msg)

    class CAR_App(AgentInterface):
        def __init__(self, agent_id, responses, sub_topic='/car_client/get_states', pub_topic='/car_client/set_states'):
            super(InterfaceDef.CAR_App, self).__init__(agent_id, responses, sub_topic, pub_topic)

    class CAR_Device(AgentInterface):
        pass
class TaskDef(object):
    """ Definitions for Task Initialisation Criteria """
    #TODO: change agent.task_details = cls.load_details(details) to call on start_task._start()

    """ Runtime Method for Custom Task Definitions """
    @classmethod
    def load_details(cls, details):
        # print('making copy...')
        c=deepcopy(details)
        # print('copy complete')
        return c
    @classmethod
    def load_task(cls, agent, task):
        agent.task_id = task['id']
        agent.task_name = task['name']
        agent.task_details = deepcopy(task['details'])
        agent.task_pointers = task['pointers'].copy()
        agent.task_stage_list = task['stage_list']
        logmsg(category="TASK", id=agent.agent_id, msg="Active task: %s" % task['name'])
        logmsg(category="TASK", msg="Task details:")
        for stage in task['stage_list']: logmsg(category="TASK", msg="    - %s" % stage)


    """ Runtime Method for Custom Task Definitions """
    @classmethod
    def generate_task(cls, agent, name, stage_list, task_id=None, details={}, pointers={}):
        #generate_task(self, "do_this", [navigate, wait, navigate], {wait_time:10}, {})
        task_name = name
        task_details = cls.load_details(details)
        task_pointers = pointers.copy()
        task_stage_list = []

        #Create dictionary for access to each stage defined in StageDef
        stage_dict = {stage:StageDef().__getattribute__(stage)
                      for stage in dir(StageDef)
                      if not stage.startswith('__')}

        #For each required stage, append the StageDef.Stage() to a list
        for S in stage_list:
            if S == "start_task":
                task_stage_list = [stage_dict[S]()]
            else:
                task_stage_list += [stage_dict[S](agent)]

        agent.task_buffer.append([task_name, task_id, task_stage_list, task_details, task_pointers])
        logmsg(category="TASK", id=agent.agent_id, msg="Buffering %s: %s" % (task_name, task_stage_list))


    """ Robot Initialisation Check """
    @classmethod
    def init_courier(cls, agent, task_id=None, details={}, pointers={}):
        task_name = "init_courier"
        task_details = cls.load_details(details)
        task_pointers = pointers.copy()
        task_stage_list = [StageDef.WaitForLocalisation(agent)]

        return({'id': task_id,
                'name': task_name,
                'details': task_details,
                'pointers': task_pointers,
                'stage_list': task_stage_list})

    @classmethod
    def idle(cls, agent, task_id=None, details={}, pointers={}):
        task_name = "idle"
        task_details = cls.load_details(details)
        task_pointers = pointers.copy()
        task_stage_list = [
            StageDef.IdleTask(agent)
        ]

        return({'id': task_id,
                'name': task_name,
                'details': task_details,
                'pointers': task_pointers,
                'stage_list': task_stage_list})

    @classmethod
    def wait_at_base(cls, agent, task_id=None, details={}, pointers={}):
        task_name = "wait_at_base"
        task_details = cls.load_details(details)
        task_pointers = pointers.copy()
        task_stage_list = [
            StageDef.AssignBaseNode(agent),
            StageDef.NavigateToBaseNode(agent),
            StageDef.IdleTask(agent)
        ]
        return({'id': task_id,
                'name': task_name,
                'details': task_details,
                'pointers': task_pointers,
                'stage_list': task_stage_list})



    """ Edge Task Template """
    @classmethod
    def edge_task(cls, agent, task_id=None, details={}, pointers={}):
        task_name = "edge_task"
        task_details = cls.load_details(details)
        task_pointers = pointers.copy()
        task_stage_list = [
            StageDef.Navigation(agent), #navigate to edge start
            StageDef.Navigation(agent)  #navigate to edge end
        ]

        return({'id': task_id,
                'name': task_name,
                'details': task_details,
                'pointers': task_pointers,
                'stage_list': task_stage_list})


    """ Task Management """
    # @classmethod
    # def pause_task(cls, agent, task_id=None, details={}, pointers={}):
    #     task_name = "pause_task"
    #     task_details = cls.load_details(details)
    #     task_pointers = pointers.copy()
    #     task_stage_list = [StageDef.Pause(agent)]
    #
    #
    #     1. #push this to start of main task
    #     2. #replace main task with this
    #     #either way, this becomed first, so do we move or remove the current task?
    # @classmethod
    # def cancel_task(cls, agent, task_id=None, details={}, pointers={}):
    #     logmsg(category="TASK", id=agent.agent_id, msg="Cancelling task %s" % (agent.task_name))
    #     task_name = ""
    #     task_details = cls.load_details(details)
    #     task_pointers = pointers.copy()
    #     task_stage_list = []
    #
    #     1. #remove ative task
    #     2. #notify each connected agent ?
    # @classmethod
    # def release_task(cls, agent, task_id=None, details={}, pointers={}):
    #     logmsg(category="TASK", id=agent.agent_id, msg="Releasing task %s" % (agent.task_name))
    #
    #     1. #?
    #
    #     #identify connected agents
    #     TaskDef.cancel_task(agent)

# class TaskDef2(object):
#     """ Definitions for Task Initialisation Criteria """
#     #TODO: change agent.task_details = cls.load_details(details) to call on start_task._start()
#
#     """ Runtime Method for Custom Task Definitions """
#     @classmethod
#     def load_details(cls, details):
#         # print('making copy...')
#         # if 'association' not in details:
#         #     details['association'] = dict()
#         c=deepcopy(details)
#         # print('copy complete')
#         return c
#     """ Runtime Method for Custom Task Definitions """
#     @classmethod
#     def generate_task(cls, agent, list, details={}):
#         agent.task_details = cls.load_details(details)
#         agent.task_stage_list = []
#
#         #Create dictionary for access to each stage defined in StageDef
#         stage_dict = {stage:StageDef().__getattribute__(stage)
#                       for stage in dir(StageDef)
#                       if not stage.startswith('__')}
#
#         #For each required stage, append the StageDef.Stage to a list
#         for S in task_stage_list:
#             if S == "start_task":
#                 agent.task_stage_list += [stage_dict[S]()]
#             else:
#                 agent.task_stage_list += [stage_dict[S](agent)]
#
#     """ Courier Initialisation Check """
#     @classmethod
#     def init_courier(cls, agent, details=dict(), task_id=None):
#         agent.task_name = "init_courier"
#         agent.task_details = cls.load_details(details)
#         agent.task_stage_list += [
#             StageDef.WaitForLocalisation(agent)
#         ]
#         logmsg(category="TASK", id=agent.agent_id, msg="Starting %s: %s" % (task_name, task_stage_list))
#
#
#     """ Initial Task Stages for Agents """
#     @classmethod
#     def idle_picker(cls, agent, details={}, task_id=None):
#         agent.task_name = "idle_picker"
#         agent.task_details = cls.load_details(details)
#         agent.task_stage_list += [
#             StageDef.IdlePicker(agent)
#         ]
#         logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
#     @classmethod
#     def idle_courier(cls, agent, details={}, task_id=None):
#         agent.task_name = "idle_courier"
#         agent.task_details = cls.load_details(details)
#         agent.task_stage_list += [
#             StageDef.AssignBaseNode(agent),
#             StageDef.NavigateToBaseNode(agent),
#             StageDef.IdleCourier(agent)
#         ]
#         logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
#     @classmethod
#     def idle_storage(cls, agent, details={}, task_id=None):
#         agent.task_name = "idle_storage"
#         agent.task_details = cls.load_details(details)
#         agent.task_stage_list += [
#             StageDef.IdleStorage(agent)
#         ]
#         logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
#
#     """ Initial Task Stages for Agents """
#     @classmethod
#     def charge_robot(cls, agent, details={}, task_id=None):
#         agent.task_name = "charge_robot"
#         agent.task_details = cls.load_details(details)
#         agent.task_stage_list += [
#             # StageDef.IdlePicker(agent)
#         ]
#         logmsg(category="TASK", id=agent.agent_id,
#                msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
#
#     """ Task Management """
#     @classmethod
#     def pause_task(cls, agent, details={}, task_id=None):
#         agent.task_name = "pause_task"
#         agent.task_details = cls.load_details(details)
#         agent.task_stage_list += [
#             StageDef.Pause(agent)
#         ]
#         logmsg(category="TASK", id=agent.agent_id,
#                msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
#     @classmethod
#     def cancel_task(cls, agent, details={}, task_id=None):
#         logmsg(category="TASK", id=agent.agent_id, msg="Cancelling task %s" % (agent.task_name))
#         agent.task_name = ""
#         agent.task_details = cls.load_details(details)
#         agent.task_stage_list = []
#     @classmethod
#     def release_task(cls, agent, details={}, task_id=None):
#         logmsg(category="TASK", id=agent.agent_id, msg="Cancelling task %s" % (agent.task_name))
#         #identify connected agents
#         TaskDef.cancel_task(agent)
class StageDef(object):
    class StageBase(object):
        def __repr__(self):
            return self.get_class()
        def get_class(self):
            return str(self.__class__).replace("<class 'rasberry_coordination.task_management.","").replace("'>","")
        def __init__(self, agent):
            self.agent = agent
            self.action_required = False
            self.route_required = False
            self.stage_complete = False
            self.new_stage = True
            self.target = None
            self.action = {}
            self.summary = {}

            self._summary()
        def _summary(self):
            placeholder = '-'
            self.summary['_start'] = placeholder
            self.summary['_notify_start'] = placeholder
            self.summary['_query'] = placeholder
            self.summary['_action'] = placeholder
            self.summary['_notify_end'] = placeholder
            self.summary['_del'] = placeholder
        def _start(self):
            """ Called whenever a new stage is set.
            Also called whenever Stage.new_stage is set to True
            """
            logmsg(category="stage", id=self.agent.agent_id, msg="Begun stage %s" % self.get_class())
            self.start_time = Time.now()
        def _notify_start(self):
            pass
        def _query(self):
            success_conditions = []  # What should be queried to tell if the stage is completed?
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            pass
        def __del__(self):
            self.stage_complete = False
            pass

    """ Standard Task Stages """
    class StartTask(StageBase):
        """ Called as the first stage of every active task (excluding idle tasks),
        this stage is responsible for the creation of the task_id on initialisation.

        Created following the convention of {agent_id}_{total_tasks++}. This is a
        unique key on condition that agents each have UUIDs for agent_id. No lock is
        required for this to fuction.

        Once this becomes the active task in the task_stage_list, the task_id is
        adopted by the agent as the active task_id.

        This stage completes without any conditions.
        """
        def __init__(self, details, task_id=None):
            super(StageDef.StartTask, self).__init__(details)
            self.task_id = task_id if task_id else "%s_%s" % (self.agent.agent_id, self.agent.total_tasks)
            self.agent.total_tasks += 1
        def _start(self):
            super(StageDef.StartTask, self)._start()
            self.agent['task_id'] = self.task_id #Set task_id as active_task_id for agent
            self.agent['start_time'] = Time.now()
        def _query(self):
            self.agent.flag(True)
        def _summary(self):
            super(StageDef.StartTask, self)._summary()
            self.summary['_start'] = "adopt active task_id"
            self.summary['_query'] = "return true"

    class WaitForLocalisation(StageBase):
        def _query(self):
            success_conditions = [self.agent.location() is not None]
            self.agent.flag(any(success_conditions))
        def __del__(self):
            super(StageDef.WaitForLocalisation, self).__del__()
            logmsg(category="stage", msg="Localisation achieved %s" % self.agent.location())

    """ Idle Placeholder Task """
    class IdleTask(StageBase):
        def _start(self):
            self.agent.task_details = {}
            super(StageDef.IdleTask, self)._start()
        def _query(self):
            success_conditions = [len(self.agent.task_buffer) > 0]
            self.agent.flag(any(success_conditions))
        def _summary(self):
            super(StageDef.IdleTask, self)._summary()
            self.summary['_start'] = 'clear task_details'
            self.summary['_query'] = 'len(task_buffer) > 0'



    """ Assignment-Based Task Stages (involves coordinator) """
    class Assignment(StageBase): #Define as ABC
        def _start(self):
            super(StageDef.Assignment, self)._start()
            self.action_required = True
        def _query(self):
            # print(self.agent.task_pointers)
            success_conditions = [self.agent.task_pointers[self.action['response_location']] != None]
            self.agent.flag(any(success_conditions))
        def _summary(self):
            super(StageDef.Assignment, self)._summary()
            self.summary['_start'] = "load service requirements"
    class AssignAgent(Assignment): pass
    class AssignNode(Assignment): pass

    class AssignWaitNode(AssignNode):
        def _start(self):
            super(StageDef.AssignWaitNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['response_location'] = 'wait_node'

            self.action['descriptor'] = 'wait_node'
            self.agent.task_pointers[self.action['response_location']] = None
    class AssignBaseNode(AssignNode):
        def _start(self):
            super(StageDef.AssignBaseNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['response_location'] = 'base_node'

            self.action['descriptor'] = 'base_node'
            self.agent.task_pointers[self.action['response_location']] = None

    """ Idle Actions for Pending Actions """
    class Idle(StageBase):
        pass

    """ Navigation Controllers for Courier """
    class Navigation(StageBase):
        def __init__(self, agent, association):
            super(StageDef.Navigation, self).__init__(agent)
            self.association = association
        def _start(self):
            super(StageDef.Navigation, self)._start()
            self.route_required = True
        def _query(self):
            success_conditions = [self.agent.location(accurate=True) == self.target]
            self.agent.flag(any(success_conditions))
        def __del__(self):
            self.agent.temp_interface.cancel_execpolicy_goal()
    class NavigateToAgent(Navigation):
        def _start(self):
            super(StageDef.NavigateToAgent, self)._start()
            self.target = self.agent.task_pointers[self.association].location()
    class NavigateToNode(Navigation):
        def _start(self):
            super(StageDef.NavigateToNode, self)._start()
            self.target = self.agent.task_pointers[self.association]

    """ Navigation SubSubclasses """
    class NavigateToBaseNode(NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToBaseNode, self).__init__(agent, association='base_node')
        def _query(self):
            success_conditions = [self.agent.location(accurate=True) == self.target]
                                  #, len(self.task_stage_list) > 2]
            self.agent.flag(any(success_conditions))
    class NavigateToWaitNode(NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToWaitNode, self).__init__(agent, association='wait_node')

    """ Active Navigation """
    class FollowAgent(StageBase):
        def _update(self):
            self.agent['target'] = self.agent['following'].location
        def _query(self):
            success_conditions = [self.agent['end_follow']]
            self.agent.flag(any(success_conditions))

    """ SSI Task Auction """
    class AwaitTaskAuction(StageBase):
        def _query(self):
            success_conditions = [self.agent['auction_to_begin'] == True]
            self.agent.flag(any(success_conditions))
    class TaskAuction(StageBase):
        def _start(self):
            self.agent['coordinator_service_required'] = True
        def _query(self):
            success_conditions = [True]
            self.agent.flag(any(success_conditions))

    """ Check Field Change """
    class CheckFieldUpdate(StageBase):
        def __init__(self, agent, field_name):
            super(StageDef.CheckFieldUpdate, self).__init__(agent)
            self.field_name = self.field_name
        def _start(self):
            self.check_value = self.agent[self.field_name]
        def _query(self):
            success_conditions = [self.check_value != self.agent[self.field_name]]
            self.agent.flag(any(success_conditions))
        def _summary(self):
            super(StageDef.CheckFieldUpdate, self)._summary()
            self.summary['__init__'] = "save field_name"
            self.summary['_start'] = "load check_value"
            self.summary['_query'] = "check field update"


    """ Meta Stages """
    class Pause(StageBase):
        def _start(self):
            self.agent.registration=False
            self.agent.task_stage_list[1]._pause()
            pass #^ # ^ self.agent.interface.cancel_execpolicy_goal()
        def _query(self):
            success_conditions = [self.registration]
            self.agent.flag(any(success_conditions))
    class Unregister(StageBase): pass

    """
    if True:

        "" Move to Target Stages ""
        class TargetedMovement(StageBase):
            def __repr__(self):
                cls_name = super(StageDef.TargetedMovement, self).__repr__()
                return "%s(%s)"%(cls_name,self.target)
            def __init__(self, agent, target):
                super(StageDef.TargetedMovement, self).__init__(agent)
                self.target = target
            def set_nav_target(self):
                self.agent.target = self.target
            def __del__(self):
                self.agent.target = None
        class Navigation(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))
        class EdgeUVTreatment(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))
        class RowUVTreatment(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))
        class EdgeDataCollection(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))
        class RowDataCollection(TargetedMovement):
            def query(self):
                success_conditions = [self.agent.current_node == self.target]
                self.agent.flag(any(success_conditions))

        "" Wait for Time Expiry Stages "
        class AwaitLoad(StageBase):
            def query(self):
                success_conditions = [self.agent.tray_loaded]
                self.agent.flag(any(success_conditions))
        class AwaitUnload(StageBase):
            def query(self):
                success_conditions = [not self.agent.tray_loaded]
                self.agent.flag(any(success_conditions))
                
                

    # "" Access Request for Storage "" #?
    # class RequestAccess(StageBase):
    #     def __call__(self):
    #         self.target = self.agent.wait_node
    #         self.agent.replan_required = True
    #     def _query(self):
    #         success_conditions = [len(self.agent.request_admittance) > 0]
    #         self.agent.flag(any(success_conditions))
    #     def __del__(self):
    #         self.agent.admittance = self.agent.request_admittance.pop(0)

                

        #"""  """
        class Wait(StageBase):
            def __init__(self, agent, timeout):
                super(StageDef.Wait, self).__init__(agent)
                self.wait_timeout = timeout
        class LoadCourier(Wait):
            def query(self):
                success_conditions = [Time.now() - self.agent.start_time > self.wait_timeout]
                self.agent.flag(any(success_conditions))
        class UnloadCourier(Wait):
            def query(self):
                success_conditions = [Time.now() - self.agent.start_time > self.wait_timeout]
                self.agent.flag(any(success_conditions))

        class Assign(StageBase):
            def __call__(self):
                self.agent.storage = closest_storage_location()
                self.agent.storage.new_task('store', {'robot': self.agent})
    """