from rasberry_coordination.task_management.modules.navigation.interfaces.GeneralNavigator import GeneralNavigator
from rasberry_coordination.robot import VirtualRobotObj


class VirtualRobot(GeneralNavigator):
    def __init__(self, agent, details):
        super(VirtualRobot, self).__init__(agent, details, VirtualRobotObj())


