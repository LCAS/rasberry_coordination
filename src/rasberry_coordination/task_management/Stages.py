""" Task Stages:

BaseStage.methods:
    __repr__: #print class
    __init__: #save agent
    __call__: #begin task
    __del__:  #del task_details
    _query:   #check completion
    _target:  #define target
    _notify:  #inform agent

BaseStage.properties:
    agent

Agent.required_properties:
    task_details

Agent.methods:
    load_idle_task()

"""
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
- modify agent.tray_present, not agent.task_stage_completed

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

class StageDef(object):
    class StageBase(object):
        def __repr__(self):
            return self.get_class()
        def get_class(self):
            return str(self.__class__).replace("<class 'rasberry_coordination.task_management.Stages.","").replace("'>","")
        def __init__(self, agent):
            self.agent = agent
            print("agent: %s begun stage %s"%(agent.agent_id,self.get_class()))
            self.new_stage = True
            self.target = None
            self._tags()
            self._summary()
        def _tags(self):
            """ Define tags used to identify certain properties about this task stage

            """
            self.active = True # not task-details since this is not cross-stage
        def _summary(self):
            self.summary = {}
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
            # self.new_stage = False #Set by run, after all start functions are called
            self.agent['coordinator_action_required'] = False
            self.agent['replan_required'] = False
            self.agent['stage_complete_flag'] = False
            self.agent['start_time'] = Time.now()
            self.agent['action_dict'] = {}
            #^ should these just be local proerties?
        def _notify_start(self):
            pass
        #we need to consider this fully
        def _update(self):
            pass
        def _query(self):
            success_conditions = []  # What should be queried to tell if the stage is completed?
            self.agent.flag = any(success_conditions)
        def _notify_end(self):
            pass
        def __del__(self):
            self.agent['stage_complete_flag'] = False
            # del self.agent.task_details
            # remove goal node?
            # remove routes?
            # communication with agent?
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
            self.agent['task_id'] = self.task_id #Assign task_id as active_task_id for agent
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
            print("localisation achieved "+self.agent.current_node)
            print("\n\n\n")

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
            self.agent['coordinator_action_required'] = True
        def _query(self):
            success_conditions = [self.agent[self.agent['action_dict']['response_location']] != None]
            self.agent.flag(any(success_conditions))
        def __del__(self):
            self.agent['coordinator_action_required'] = False
        def _summary(self):
            super(StageDef.Assignment, self)._summary()
            self.summary['_start'] = "load service requirements"
            self.summary['_del'] = "coordinator_action_required = False"

    class AssignCourier(Assignment):
        def _start(self):
            super(StageDef.AssignCourier, self)._start() #defined as default setup
            self.agent['action_dict']['action_type'] = 'find_agent'
            self.agent['action_dict']['action_style'] = 'closest'
            self.agent['action_dict']['response_location'] = 'courier'
            self.agent['action_dict']['agent_type'] = 'robot'  # local_storage/cold_storage
        def _notify_end(self):
            self.agent.interface.notify("ACCEPT")
        def __del__(self):
            """ On completion of assign courier, a courier should have been identified.
            As a result of the completion, the couier should be assigned a task, and be
            given details pertaining to its completion. This is done explicitly.
            """
            super(StageDef.AssignCourier, self).__del__()
            self.agent['courier'].start_new_task(details={'target_agent':self.agent},
                                                 task_id=self.agent['task_id'])
        def _summary(self):
            super(StageDef.AssignCourier, self)._summary()
            self.summary['_query'] = "agent[courier] not None"
            self.summary['_action'] = "find closest robot courier"
            self.summary['_del'] = "agent[courier][target]=agent"
            self.summary['_notify_end'] = "agent -> ACCEPT"
    class AssignStorage(Assignment):
        def _start(self):
            super(StageDef.AssignStorage, self)._start()
            self.agent['action_dict']['action_type'] = 'find_agent'
            self.agent['action_dict']['action_style'] = 'closest'
            self.agent['action_dict']['response_location'] = 'storage'

            self.agent['action_dict']['agent_type'] = 'storage' #local_storage/cold_storage
        def __del__(self):
            super(StageDef.AssignStorage, self).__del__()
            self.agent['storage'].request_admittance.append(self.agent.agent_id)
    class AssignWaitNode(Assignment):
        def _start(self):
            super(StageDef.AssignWaitNode, self)._start()
            self.agent['action_dict']['action_type'] = 'find_node'
            self.agent['action_dict']['action_style'] = 'closest'
            self.agent['action_dict']['response_location'] = 'wait_node'

            self.agent['action_dict']['descriptor'] = 'wait_node'  # local_storage/cold_storage
    class AssignBaseStationNode(Assignment):
        def _start(self):
            super(StageDef.AssignBaseStationNode, self)._start()
            self.agent['action_dict']['action_type'] = 'find_node'
            self.agent['action_dict']['action_style'] = 'closest'
            self.agent['action_dict']['response_location'] = 'base_station'

            self.agent['action_dict']['descriptor'] = 'base_station'  # local_storage/cold_storage

    class AcceptCourier(Assignment):
        def _start(self):
            super(StageDef.AcceptCourier, self)._start()
            self.agent['action_dict']['action_type'] = 'find_agent_from_list'
            self.agent['action_dict']['action_style'] = 'closest'
            self.agent['action_dict']['response_location'] = 'courier'

            self.agent['action_dict']['list'] = self.agent.request_admittance
        def __del__(self):
            super(StageDef.AcceptCourier, self).__del__()
            logmsg(category="stage", msg="Admitted: %s from %s" % (self.agent['courier'].agent_id, self.agent.request_admittance))
            self.agent.request_admittance.remove(self.agent['courier'].agent_id)
        #On completion: courier being accepted is found @ agent['courier']
        # def __del__(self):
        #     print("\n\n\n")
        #     print("AcceptCourier __del__()")
        #     print(self.agent['courier'].__dict__)
        #     print("\n-----\n")
        #     print(self.agent.__dict__)
        #     print("\n\n\n-----")

    """ Idle Actions for Pending Actions """
    class Idle(StageBase):
        pass
    class AwaitCourier(Idle): #PICKER + STORAGE
        def _query(self):
            success_conditions = [self.agent['courier'].location() == self.agent.location()]
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            self.agent.interface.notify("ARRIVED")
        def _summary(self):
            super(StageDef.AwaitCourier, self)._summary()
            self.summary['_query'] = "courier @ agent"
            self.summary['_notify_end'] = "agent -> ARRIVED"

    class AwaitStoreAccess(Idle):
        #While waiting for store access, move to a wait_node
        # (ideally this should be identified by the coordinator and assigned dynamically)
        def _start(self):
            super(StageDef.AwaitStoreAccess, self)._start()
            self.agent['replan_required'] = True
            # Despite moving to a wait node, dont end task on arrival #huh?

            # print("\n\n\n")
            # print("AwaitStoreAccess _Start()")
            # print(self.agent['storage'].__dict__)
            # print("\n-----\n")
            # print(self.agent.__dict__)
            # print("\n\n\n-----")

        def _query(self):
            courier = self.agent['storage']['courier']
            if not courier:
                return
            success_conditions = [courier.agent_id == self.agent.agent_id]
            self.agent.flag(any(success_conditions))
        # def __del__(self):
        #     print("await_store stage completed")
        #     print(self.agent['storage'].__dict__)

    """ Navigation Controllers for Courier """
    class Navigation(StageBase):
        def _start(self):
            super(StageDef.Navigation, self)._start()
            self.agent['replan_required'] = True
        def __del__(self):
            self.agent.temp_interface.cancel_execpolicy_goal()

    """ Agent Navigation """
    class NavigateToAgent(Navigation):
        def _start(self):
            super(StageDef.NavigateToAgent, self)._start()
            self.target = self.agent['target_agent'].location()
        def _query(self):
            # self.target = self.agent['target_agent'].location()
            success_conditions = [self.agent.location(accurate=True) == self.target]
                                # , self.agent.execpolicy.success]
            self.agent.flag(any(success_conditions))
    class NavigateToPicker(NavigateToAgent):
        pass
    class NavigateToStorage(NavigateToAgent):
        def _start(self):
            super(StageDef.NavigateToStorage, self)._start()
            self.target = self.agent['storage'].location()

    """ Node Navigation """
    class NavigateToNode(Navigation):
        def __init__(self, agent, target_identifier):
            super(StageDef.NavigateToNode, self).__init__(agent)
            self.target_identifier = target_identifier
        def _start(self):
            super(StageDef.NavigateToNode, self)._start()
            self.target = self.agent[self.target_identifier]
        def _query(self):
            success_conditions = [self.agent.location(accurate=True) == self.target]
            # , self.agent.execpolicy.success]
            self.agent.flag(any(success_conditions))
    class NavigateToBaseStationNode(NavigateToNode):
        def __init__(self, agent):
            super(StageDef.NavigateToBaseStationNode, self).__init__(agent, target_identifier='base_station')
    class NavigateToWaitNode(Navigation):
        def __init__(self, agent):
            super(StageDef.StartTask, self).__init__(agent, target_identifier='wait_node')


    """ Active Navigation """
    class FollowAgent(StageBase):
        def _update(self):
            self.agent['target'] = self.agent['following'].location
        def _query(self):
            success_conditions = [self.agent['end_follow']]
            self.agent.flag(any(success_conditions))

    """ Loading Modifiers for Picker and Storage """
    class LoadModifier(StageBase):
        def __del__(self):
            super(StageDef.LoadModifier, self)._start()
            # print("setting courier['tray_present'] to %s" % not self.end_requirement)
            self.agent['courier']['tray_present'] = not self.end_requirement
    class LoadCourier(LoadModifier): #PICKER
        def __init__(self, agent):
            super(StageDef.LoadCourier, self).__init__(agent)
            self.wait_timeout = Duration(secs=5)
            self.end_requirement = False #flag must be this to end task
            self.agent['picker_has_tray'] = True #local flag
        def _query(self):
            success_conditions = [Time.now() - self.agent['start_time'] > self.wait_timeout,
                                 self.agent['picker_has_tray'] == self.end_requirement]
            self.agent.flag(any(success_conditions))
        def _notify_end(self):
            self.agent.interface.notify("INIT")
    class UnloadCourier(LoadModifier): #STORAGE
        def __init__(self, agent):
            super(StageDef.UnloadCourier, self).__init__(agent)
            self.wait_timeout = Duration(secs=5)
            self.end_requirement = True #flag must be this to end task
            self.agent['storage_has_tray'] = False  # local flag
        def _query(self):
            success_conditions = [Time.now() - self.agent['start_time'] > self.wait_timeout,
                                  self.agent['storage_has_tray'] == self.end_requirement]
            self.agent.flag(any(success_conditions))


    """ Loading Modifiers for Courier """
    class Loading(StageBase):
        def _query(self):
            success_conditions = [self.agent['tray_present']]
            # if any(success_conditions):
            #     print("Loading stage complete")
            self.agent.flag(any(success_conditions))
    class Unloading(StageBase):
        def _query(self):
            success_conditions = [not self.agent['tray_present']]
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