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
        def _summary(self):
            super(StageDef.CheckFieldUpdate, self)._summary()
            self.summary['__init__'] = "save field_name"
            self.summary['_start'] = "load check_value"
            self.summary['_query'] = "check field update"


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

