from rasberry_coordination.task_management.modules.navigation.interfaces.GeneralNavigator import GeneralNavigator
from rasberry_coordination.robot import Robot as RobotObj


class Robot(GeneralNavigator):
    def __init__(self, agent, details):
        super(Robot, self).__init__(agent, details, RobotObj(agent.agent_id, agent.speaker))


