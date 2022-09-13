"""Base"""

from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, get_param
from std_msgs.msg import Bool, String as Str
from diagnostic_msgs.msg import KeyValue
import strands_executive_msgs.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.action_management.manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.robot import Robot, VirtualRobot
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class InterfaceDef(object):

    class robot(object):
        def __init__(self, agent, Type):
            self.agent = agent
            self.agent.navigation_interface = Type
            self.exit_at_node_sub = Subscriber('/%s/base/exit_at_node' % agent.agent_id, Str, self.exit_at_node_cb)
            self.move_idle_sub = Subscriber('/%s/base/move_idle' % agent.agent_id, Str, self.move_idle_cb)
        def exit_at_node_cb(self, msg):
            logmsg(category="Task", id=self.agent.agent_id, msg="Request to exit coordinator")
            node_id = msg.data or self.agent.goal or self.agent.location(accurate=True)
            self.agent.add_task(task_name='exit_at_node', contacts={"exit_node":node_id}, index=0)
        def move_idle_cb(self, msg):
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
            self.agent.add_task(task_name='move_idle', contacts={"target":msg.data})

    class base_robot(robot):
        def __init__(self, agent):
            super(InterfaceDef.base_robot, self).__init__(agent, Robot(agent.agent_id, agent.speaker))
    class base_virtual_robot(robot):
        def __init__(self, agent):
            sd = fetch_property('base', 'virtual_robot_step_delay')
            VR = VirtualRobot(agent.agent_id, agent, step_delay=sd)
            super(InterfaceDef.base_virtual_robot, self).__init__(agent, VR)


    class base_human(object):
        def __init__(self, agent):
            self.agent = agent
    class base_localised_human(base_human):
        pass

