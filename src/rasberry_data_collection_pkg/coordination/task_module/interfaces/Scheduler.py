from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages
from rasberry_coordination.coordinator_tools import logmsg

class Scheduler(Interface):

    def assign_scan_row(self, task_id=None, details=None, contacts=None, initiator_id=""):
        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduled assignment for a row treatment")
        return(Task(id=task_id,
                    module='rasberry_data_collection_pkg',
                    name="assign_scan_row",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['rasberry_data_collection_pkg']['ScheduleScanner'](self.agent, details, 'scheduled_scan_row')
                    ]))

    def assign_scan_edge(self, task_id=None, details=None, contacts=None, initiator_id=""):
        #details = details or dict()
        #details["action_time_limit"] = 100 #TODO what fallout arises from fields here moving towards start of pipeline?

        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduled assignment for a edge treatment")
        return(Task(id=task_id,
                    module='rasberry_data_collection_pkg',
                    name="assign_scan_edge",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        #Stages['assignment']['StartAuction'](self.agent, details),
                        Stages['rasberry_data_collection_pkg']['ScheduleScanner'](self.agent, details, 'scheduled_scan_edge')
                    ]))
