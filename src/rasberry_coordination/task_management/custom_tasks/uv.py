from std_msgs.msg import String as Str
from rasberry_coordination.coordinator_tools import logmsg
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.task_management.base import StageDef as RootDef, InterfaceDef as CommsDef
from rasberry_coordination.robot import Robot as RobotInterface_Old

class InterfaceDef(object):
    class uv_phototherapist(CommsDef.AgentInterface):
        def __init__(self, agent, sub='/lar/get_states', pub='/lar/set_states'):
            responses = {}
            super(InterfaceDef.uv_phototherapist, self).__init__(agent, responses, sub=sub, pub=pub)

    class uv_controller(CommsDef.AgentInterface):
        def __init__(self, agent, sub='/lar/get_states', pub='/lar/set_states'):
            responses = {}
            super(InterfaceDef.uv_controller, self).__init__(agent, responses, sub=sub, pub=pub)


class TaskDef(object):
    pass

class StageDef(object):
    pass