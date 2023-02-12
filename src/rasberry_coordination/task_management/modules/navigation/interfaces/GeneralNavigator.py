from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from rasberry_coordination.coordinator_tools import logmsg

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages

from rasberry_coordination.topomap_management.occupancy import OccupancyFilters

from rospy import Subscriber
from std_msgs.msg import String as Str

class GeneralNavigator(Interface):

    def __init__(self, agent, details):
        super(GeneralNavigator, self).__init__(agent, details)
        self.move_idle_sub = Subscriber('/%s/navigation/move_idle' % agent.agent_id, Str, self.wait_at_node_cb)
        self.exit_at_node_sub = Subscriber('/%s/navigation/exit_at_node' % agent.agent_id, Str, self.exit_at_node_cb)
        self.occupation_type = self.details['occupation'] if 'occupation' in self.details else None

    def occupation(self):
        """ Filter map based on occupation types associated to node name """

        # Early return if agent has no presence
        if not self.agent.location.has_presence:
            logmsg(category="occupy", msg="   :   | no presence")
            return []

        #Get location name
        node = self.agent.location(accurate=False)

        # Early return if agent has no location
        if not node:
            logmsg(category="occupy", msg="   :   | no location")
            return []

        # Early return if agent has no map
        if not self.agent.map_handler.global_node_list:
            logmsg(category="occupy", msg="   :   | no map")
            return []

        # Find filter type
        occ_type = 'self'
        if node and node.startswith('dock_'):
            occ_type = 'dock-'
        elif node and node.startswith('r') and '-c' in node:
            occ_type = 'r-c'

        # Find occupation type to apply
        type_list = ['self']
        if occ_type in self.occupation_type:
            type_list = self.occupation_type[occ_type]

        # Apply filters
        nodes_to_filter = []
        logmsg(category="occupy", msg="   | %s"%self.agent.agent_id)
        for typ in type_list:
            method = getattr(OccupancyFilters, typ)
            nodes = method(self.agent.map_handler.global_map, self.agent.map_handler.global_node_list, node)
            nodes.sort()
            nodes_to_filter += nodes
            logmsg(category="occupy", msg="   :   | %s: %s"%(typ, str(nodes)))
        return nodes_to_filter

    def wait_at_node_cb(self, msg):
        logmsg(category="Task", id=self.agent.agent_id, msg="Request to Move Idle to (%s)"%msg.data)

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






    def occupation_test(self):
        def do(t, n):
            logmsg(category="occupy", msg="Test %s (%s)"%(t, n))
            m = getattr(OccupancyFilters, t)
            l = m(self.agent.map_handler.global_map, self.agent.map_handler.global_node_list, n)
            logmsg(category="occupy", msg="    -> %s"%(str(l)))
            print("|")

        T = ["neighbour_short_ends", "neighbour_tall_ends"]
        R = ['r0.7-c2', 'r1-c2', 'r1.5-c2', 'r2-c2', 'r2.5-c2', 'r3-c2', 'r3.5-c2', 'r4-c2', 'r4.5-c2', 'r5.3-c2', 'r5.7-c2']
        for t in T:
            for r in R:
                do(t,r)
        quit()


