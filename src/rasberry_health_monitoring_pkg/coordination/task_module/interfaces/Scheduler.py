from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages
from rasberry_coordination.coordinator_tools import logmsg

class Scheduler(Interface):

    def assign_send_for_mot(self, task_id=None, details=None, contacts=None, initiator_id=""):
        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduled assignment for a mot")
        if not details['msg'].criteria.viable_agents: return
        return(Task(id=task_id,
                    module='rasberry_health_monitoring_pkg',
                    name="send_for_mot",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        #Stages['rasberry_health_monitoring_pkg']['AssignRobot'](self.agent, details, 'send_for_mot'),
                    ]))

    def assign_stretch_legs(self, task_id=None, details=None, contacts=None, initiator_id=""):
        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduled assignment for a stretch")
        if not details['msg'].criteria.viable_agents: return
        return(Task(id=task_id,
                    module='rasberry_health_monitoring_pkg',
                    name="stretch_legs",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                    ]))



