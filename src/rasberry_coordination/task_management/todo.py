#File to store planned core-tasks
from pprint import pprint
from copy import deepcopy
from std_msgs.msg import String as Str
import strands_executive_msgs.msg
import std_msgs.msg
from std_msgs.msg import Bool
import std_srvs.srv
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy
from rasberry_coordination.coordinator_tools import logmsg
import rasberry_coordination.srv
import rasberry_coordination.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption


class InterfaceDef(object):
    class CAR_App(AgentInterface): pass
    class CAR_Device(AgentInterface): pass

class TaskDef(object):

    @classmethod
    def generate_task(cls, agent, name, stage_list, task_id=None, details={}, contacts={}):
        #generate_task(self, "do_this", [navigate, wait, navigate], {wait_time:10}, {})
        task_name = name
        task_details = cls.load_details(details)
        task_contacts = contacts.copy()
        task_module = 'base'
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

        agent.task_buffer.append([task_name, task_id, task_stage_list, task_details, task_contacts])
        logmsg(category="TASK", id=agent.agent_id, msg="Buffering %s: %s" % (task_name, task_stage_list))



    """ Edge Task Template """
    @classmethod
    def edge_task(cls, agent, task_id=None, details={}, contacts={}):
        task_name = "edge_task"
        task_details = cls.load_details(details)
        task_contacts = contacts.copy()
        task_module = 'base'
        task_stage_list = [
            StageDef.Navigation(agent), #navigate to edge start
            StageDef.Navigation(agent)  #navigate to edge end
        ]

        return({'id': task_id,
                'name': task_name,
                'details': task_details,
                'contacts': task_contacts,
                'task_module': task_module,
                'stage_list': task_stage_list})



class StageDef(object):
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
            def _end(self):
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
    #     def _end(self):
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

""" Task Stages:
- All stages must inherit from StageBase.
"""

# Standard Stage Methods:
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

    - name: _end
      description:
      super: True

    - name: __repr__
      description: 
    - name: _get_class
      description: 
"""

# Standard Stage Attributes:
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

    - name: start_time
      description: 


"""

# Subclass Inheritance Tree
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
- all overloading of _end must include a call to super

Notes:
- __init__ is called when the task stages are added to an agent
- __repr__ can be overloaded to include more information
- _start is called once when the stask is at the head of task stage_list
- _start can be called a second time by setting Stage.new_stage to True
- _query is called on every iteration of Coordinator.run, except when task_progression is paused
- 

(Tim Peters, The Zen of Python):
"Special cases aren't special enough to break the rules." / "Although practicality beats purity."
-> `python -c "import this"`
"""