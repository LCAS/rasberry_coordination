from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_scheduling.msg import TaskContents

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages
from rasberry_coordination.coordinator_tools import logmsg

class Scheduler(Interface):

    @classmethod
    def list_schedulable_tasks(cls):
        t1 = TaskContents(task_name='scan_row', nodes=['r1-c1'])
        t2 = TaskContents(task_name='scan_edge', nodes=['r1-cb', 'r1-c0'])
        return [t1,t2]

    def assign_scan_row(self, task_id=None, details=None, contacts=None, initiator_id=""):
        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduled assignment for: %s"%details['msg'].criteria.task_name)

        if not details['msg'].criteria.nodes:
            logmsg(category="SCHEDU", msg="    | no row id included")
            return

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

        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduled assignment for: %s"%details['msg'].criteria.task_name)

        nodes = details['msg'].criteria.nodes
        if not len(nodes) == 2:
            logmsg(level='error', category="SCHEDU", msg="    | lacking 2 ends of edge regino included")
            return
        if nodes[0].split('-')[0] != nodes[1].split('-')[0]:
            logmsg(level='error', category="SCHEDU", msg="    | nodes are not of same row")
            return

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
