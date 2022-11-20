from rospy import get_param
from rasberry_coordination.task_management.modules.navigation.interfaces.GeneralNavigator import GeneralNavigator
from rasberry_coordination.robot import VirtualRobot as VirtualRobotObj


class RobotVirtual(GeneralNavigator):
    def __init__(self, agent, details):
        super(RobotVirtual, self).__init__(agent, details, VirtualRobotObj(agent.agent_id, agent))
