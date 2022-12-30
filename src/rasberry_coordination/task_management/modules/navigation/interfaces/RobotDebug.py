from rospy import get_param
from rasberry_coordination.task_management.modules.navigation.interfaces.GeneralNavigator import GeneralNavigator
from rasberry_coordination.debugrobot import DebugRobot as DebugRobotObj


class RobotDebug(GeneralNavigator):
    def __init__(self, agent, details):
        super(RobotDebug, self).__init__(agent, details, DebugRobotObj(agent))
