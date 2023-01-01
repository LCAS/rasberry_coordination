from rasberry_coordination.task_management.__init__ import Stages
from rasberry_coordination.task_management.containers.Task import TaskObj as Task

class Interface(object):

    def __repr__(self):
        return str(self.__class__).split('\'')[1].split('.')[-1]

    def __init__(self, agent, details=None):
        self.agent = agent
        self.details = details
        self.name = self.__repr__()

    def init(self, task_id=None, details=None, contacts=None, initiator_id=""):
        pass

    def idle(self, task_id=None, details=None, contacts=None, initiator_id=""):
        if len(self.agent.task_buffer) == 0:
            return(Task(id=task_id,
                        module='base',
                        name="idle",
                        details=details,
                        contacts=contacts,
                        initiator_id=self.agent.agent_id,
                        responder_id="",
                        stage_list=[
                            Stages['base']['StartTask'](self.agent, task_id),
                            Stages['base']['Idle'](self.agent)
                        ]))
