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

from rospy import Time, Duration
from rasberry_coordination.coordinator_tools import logmsg

class TaskDef(object):
    """ Definitions for Task Initialisation Criteria """


    """ Runtime Method for Custom Task Definitions """
    @classmethod
    def load_details(cls, details):
        return details.copy()

    """ Runtime Method for Custom Task Definitions """
    @classmethod
    def generate_task(cls, agent, list, details={}):
        agent.task_details = cls.load_details(details)
        agent.task_stage_list = []

        #Create dictionary for access to each stage defined in StageDef
        stage_dict = {stage:StageDef().__getattribute__(stage)
                      for stage in dir(StageDef)
                      if not stage.startswith('__')}

        #For each required stage, append the StageDef.Stage to a list
        for S in task_stage_list:
            if S == "start_task":
                agent.task_stage_list += [stage_dict[S]()]
            else:
                agent.task_stage_list += [stage_dict[S](agent)]

    """ Courier Initialisation Check """
    @classmethod
    def init_courier(cls, agent, details={}, task_id=None):
        agent.task_name = "init_courier"
        agent.task_details = cls.load_details(details)
        agent.task_stage_list += [
            StageDef.WaitForLocalisation(agent)
        ]
        logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))

    """ Initial Task Stages for Agents """
    @classmethod
    def idle_picker(cls, agent, details={}, task_id=None):
        agent.task_name = "idle_picker"
        agent.task_details = cls.load_details(details)
        agent.task_stage_list += [
            StageDef.IdlePicker(agent)
        ]
        logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
    @classmethod
    def idle_courier(cls, agent, details={}, task_id=None):
        agent.task_name = "idle_courier"
        agent.task_details = cls.load_details(details)
        agent.task_stage_list += [
            StageDef.AssignBaseNode(agent),
            StageDef.NavigateToBaseNode(agent),
            StageDef.IdleCourier(agent)
        ]
        logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))
    @classmethod
    def idle_storage(cls, agent, details={}, task_id=None):
        agent.task_name = "idle_storage"
        agent.task_details = cls.load_details(details)
        agent.task_stage_list += [
            StageDef.IdleStorage(agent)
        ]
        logmsg(category="TASK", id=agent.agent_id, msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))

    """ Initial Task Stages for Agents """
    @classmethod
    def charge_robot(cls, agent, details={}, task_id=None):
        agent.task_name = "charge_robot"
        agent.task_details = cls.load_details(details)
        agent.task_stage_list += [
            # StageDef.IdlePicker(agent)
        ]
        logmsg(category="TASK", id=agent.agent_id,
               msg="Beginning %s: %s" % (agent.task_name, agent.task_stage_list))


class StageDef(object):
    class StageBase(object):
        def __repr__(self):
            return self.get_class()
        def get_class(self):
            return str(self.__class__).replace("<class 'rasberry_coordination.task_management.Stages.","").replace("'>","")
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
            logmsg(category="stage", msg="Localisation achieved "+self.agent.current_node)

    """ Idle Placeholder Task """
    class IdleTask(StageBase):
        def _start(self):
            self.agent.task_details = {}
            super(StageDef.IdleTask, self)._start()
        def _query(self):
            success_conditions = [len(self.agent.task_stage_list) > 1
                                 #,self.agent['begin_task'] == "transportation_picker"
                                 ]
            self.agent.flag(any(success_conditions))
        def _summary(self):
            super(StageDef.IdleTask, self)._summary()
            self.summary['_start'] = 'clear task_details'
            self.summary['_query'] = 'len(task_stage_list) > 1'

    class IdlePicker(IdleTask):
        """ Assigned to any picker agents with an empty task_stage_list
        this task empties task_details no longer needed.

        Once the idle picker has been assigned a task, the task_stage_list
        will have additional stages. On this conditional, the idle task will end.
        """
        pass
    class IdleCourier(IdleTask):
        # def __del__(self):
        #     self.agent.start_new_task(self.agent['task_type'],
        #                         {'target_agent': self.agent['target']})
        pass
    class IdleStorage(IdleTask):
        def _query(self):
            success_conditions = [len(self.agent.request_admittance) > 0] #TODO: this may prove error prone w/ _start
            self.agent.flag(any(success_conditions))
        def __del__(self):
            self.agent.start_new_task('transportation_storage', {'target_agent': self.agent})
        def _summary(self):
            super(StageDef.IdleStorage, self)._summary()
            self.summary['_query'] = 'len(store.request_admittance) > 0'
            self.summary['_del'] = 'begin task'


    """ Assignment-Based Task Stages (involves coordinator) """
    class Assignment(StageBase): #Define as ABC
        def _start(self):
            super(StageDef.Assignment, self)._start()
            self.action_required = True
        def _query(self):
            success_conditions = [self.agent[self.action['response_location']] != None]
            self.agent.flag(any(success_conditions))
        def _summary(self):
            super(StageDef.Assignment, self)._summary()
            self.summary['_start'] = "load service requirements"

    class AssignWaitNode(Assignment):
        def _start(self):
            super(StageDef.AssignWaitNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['response_location'] = 'wait_node'

            self.action['descriptor'] = 'wait_node'  # local_storage/cold_storage
    class AssignBaseNode(Assignment):
        def _start(self):
            super(StageDef.AssignBaseNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['response_location'] = 'base_node'

            self.action['descriptor'] = 'base_node'  # local_storage/cold_storage

    """ Idle Actions for Pending Actions """
    class Idle(StageBase):
        pass

    """ Navigation Controllers for Courier """
    class Navigation(StageBase):
        def __init__(self, agent, target_identifier):
            super(StageDef.Navigation, self).__init__(agent)
            self.target_identifier = target_identifier
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
            self.target = self.agent[self.target_identifier].location()
    class NavigateToNode(Navigation):
        def _start(self):
            super(StageDef.NavigateToNode, self)._start()
            self.target = self.agent[self.target_identifier]

    """ Navigation SubSubclasses """
    class NavigateToBaseNode(NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToBaseNode, self).__init__(agent, target_identifier='base_node')
    class NavigateToWaitNode(NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToWaitNode, self).__init__(agent, target_identifier='wait_node')

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


    """ Meta Stages """
    class Pause(StageBase):
        def _start(self):
            self.agent.registration=False
            self.agent.task_stage_list[1]._pause()
            pass #^ # ^ self.agent.interface.cancel_execpolicy_goal()
        def _query(self):
            success_conditions = [self.registration]
            self.agent.flag(any(success_conditions))


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