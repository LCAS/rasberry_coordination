from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.StateInterface import StateInterface
from rasberry_coordination.task_management.__init__ import Stages


class Controller(StateInterface):

    def sar_BEGUN(self):
        task_scope, details = self.get_task('rasberry_data_collection_pkg')
        if task_name and details: self.agent.add_task(module='rasberry_data_collection_pkg', name='command_scan', details=details)

    def sar_CANCEL(self):
        if self.agent['name'] == 'command_scan':
            logmsg(level="error", category="IDef", id=self.agent.agent_id, msg="already has task")
            self.agent.set_interrupt('reset', 'rasberry_data_collection_pkg', self.agent['id'], "Task")

    def sar_EMERGENCY_STOP(self):
        if self.agent['name'] == 'command_scan':
            self.agent.set_interrupt('pause', 'rasberry_data_collection_pkg', self.agent['id'], "Task")
            if 'scanner' in self.agent['contacts'] and 'Pause' not in self.agent['contacts']['scanner']().get_class():
               self.agent['contacts']['scanner'].set_interrupt('pause', 'rasberry_data_collection_pkg', self.agent['id'], "Task")

    def sar_EMERGENCY_RESUME(self):
        if self.agent['name'] == 'command_scan':
            self.agent.set_interrupt('resume', 'rasberry_data_collection_pkg', self.agent['id'], "Task")
            if 'scanner' in self.agent['contacts'] and 'Pause' in self.agent['contacts']['scanner']().get_class():
                self.agent['contacts']['scanner'].set_interrupt('resume', 'rasberry_data_collection_pkg', self.agent['id'], "Task")

    def command_scan(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_data_collection_pkg',
                    name="command_scan",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['rasberry_data_collection_pkg']['AssignScanner'](self.agent, details),
                        Stages['rasberry_data_collection_pkg']['AwaitCompletion'](self.agent),
                    ]))

    def get_task(self, module):
        state, row, edge, task, robot = self.msg.value.split('-')
        nodes = []
        if task != module: return (None,None)
        elif edge != "all":
            task_scope = 'edge'
            nodes = ["r%s-c%s" % (row, e) for e in edge.split('>')]
        elif row != "all":
            task_scope = 'row'

        return (task_scope, {'row': 'r'+row,
                             'edge': edge,
                             'nodes': nodes,
                             'robot': robot,
                             'scope': task_scope})

