from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from rasberry_coordination.coordinator_tools import logmsg

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages

from rospy import Subscriber
from std_msgs.msg import String as Str

class GeneralNavigator(Interface):

    def __init__(self, agent, details):
        super(GeneralNavigator, self).__init__(agent, details)
        self.move_idle_sub = Subscriber('/%s/base/move_idle' % agent.agent_id, Str, self.wait_at_node_cb)
        self.exit_at_node_sub = Subscriber('/%s/base/exit_at_node' % agent.agent_id, Str, self.exit_at_node_cb)
        self.occupation_type = self.details['occupation'] if 'occupation' in self.details else None

    def occupation(self):
        """ Filter map based on occupation types associated to node name """
        if self.agent.location.has_presence:

            #Get location name
            name = self.agent.location(accurate=False)

            # Find filter types to apply
            if node.startswith('dock_'):
                type_list = self.occupation_type['dock_*']
            elif node.startswith('r') and '-c' in node:
                type_list = self.occupation_type['r*-c*']
            else:
                type_list = ["self"]
            # Apply filters
            nodes_to_filter = []
            for typ in type_list:
                method = getattr(OccupancyFilters, typ)
                nodes_to_filter += method(self.agent.map_handler.empty_map, self.agent.map_handler.empty_node_list, node)
        return nodes_to_filter

    def wait_at_node_cb(self, msg):
        logmsg(category="Task", id=self.agent.agent_id, msg="Request to Move while Idle")

        # if idle:
        if not isinstance(self.agent(), Stages['base']['Idle']):
            logmsg(category="Task", msg="    - agent not idle")
            return

        # if node valid:
        if not self.agent.map_handler.is_node(msg.data):
            logmsg(category="Task", msg="    - node is invalid")
            return

        # add task
        self.agent.add_task(module='navigation', name='wait_at_node', contacts={"target": msg.data})

    def wait_at_node(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='navigation',
                    name="wait_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent, task_id),
                        Stages['navigation']['NavigateToNode'](self.agent, 'target'),
                        Stages['base']['Idle'](self.agent)
                    ]))



    def exit_at_node_cb(self, msg):
        logmsg(category="Task", id=self.agent.agent_id, msg="Request to exit coordinator")
        node_id = msg.data or self.agent.goal or self.agent.location(accurate=True)
        self.agent.add_task(module='navigation', name='exit_at_node', contacts={"node_contact_id":node_id}, index=0)

    def exit_at_node(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='navigation',
                    name="exit_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['SetUnregister'](self.agent),
                        Stages['navigation']['NavigateToNode'](self.agent, contact_id='node_contact_id'),
                        Stages['base']['Exit'](self.agent)
                    ]))

