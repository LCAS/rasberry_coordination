from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_scheduling.msg import TaskContents

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages
from rasberry_coordination.coordinator_tools import logmsg

class Scheduler(Interface):

    @classmethod
    def list_schedulable_tasks(cls):
        t1 = TaskContents(task_name='send_for_mot', viable_agents=['thorvald_001'], nodes=['WayPoint140'])
        return [t1]

    def assign_send_for_mot(self, task_id=None, details=None, contacts=None, initiator_id=""):
        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduled assignment for: %s"%details['msg'].criteria.task_name)
        if not details['msg'].criteria.viable_agents:
            logmsg(category="SCHEDU", msg="    | no agent included")
            return
        if not details['msg'].criteria.nodes:
            logmsg(category="SCHEDU", msg="    | no location included")
            return
        return(Task(id=task_id,
                    module='rasberry_health_monitoring_pkg',
                    name="send_for_mot",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['rasberry_health_monitoring_pkg']['AssignRobot'](self.agent, details, 'send_for_mot'),
                    ]))
