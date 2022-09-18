from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages

from rospy import Subscriber
from std_msgs.msg import String as Str

class GeneralNavigator(Interface):

    def __init__(self, agent, details, Type):
        super(GeneralNavigator, self).__init__(agent, details)
        self.agent.navigation_interface = Type
        self.move_idle_sub = Subscriber('/%s/base/move_idle' % agent.agent_id, Str, self.wait_at_node_cb)
        self.exit_at_node_sub = Subscriber('/%s/base/exit_at_node' % agent.agent_id, Str, self.exit_at_node_cb)

    def wait_at_node_cb(self, msg):
        logmsg(category="Task", id=self.agent.agent_id, msg="Request to Move while Idle")

        # if idle:
        if not isinstance(self.agent(), StageDef.Idle):
            logmsg(category="Task", msg="    - agent not idle")
            return

        # if node valid:
        if not self.agent.map_handler.is_node(msg.data):
            logmsg(category="Task", msg="    - node is invalid")
            return

        # add task
        self.agent.add_task(module='navigation', name='wait_at_node', contacts={"target": msg.data})

    def wait_at_node(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='navigation',
                    name="wait_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['assignment']['AssignNode'](self.agent),
                        Stages['navigation']['NavigateToNode'](self.agent),
                        Stages['base']['Idle'](self.agent)
                    ]))



    def exit_at_node_cb(self, msg):
        logmsg(category="Task", id=self.agent.agent_id, msg="Request to exit coordinator")
        node_id = msg.data or self.agent.goal or self.agent.location(accurate=True)
        self.agent.add_task(task_name='exit_at_node', contacts={"exit_node":node_id}, index=0)

    def exit_at_node(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='navigation',
                    name="exit_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['SetUnregister'](self.agent),
                        Stages['navigation']['NavigateToExitNode'](self.agent),
                        Stages['base']['Exit'](self.agent)
                    ]))

