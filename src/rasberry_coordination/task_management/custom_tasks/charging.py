from std_msgs.msg import String as Str
from rasberry_coordination.coordinator_tools import logmsg
from rospy import Time, Duration, Subscriber, Publisher, Time
from rasberry_coordination.task_management.base import StageDef as RootDef, InterfaceDef as CommsDef
from rasberry_coordination.robot import Robot as RobotInterface_Old

class InterfaceDef(object): pass
class TaskDef(object):

    """ Initial Task Stages for Agents """
    @classmethod
    def charge_robot(cls, agent, task_id=None, details={}, pointers={}):
        task_name = "charge_robot"
        task_details = cls.load_details(details)
        task_pointers = pointers.copy()
        task_stage_list = [
            # StageDef.IdlePicker(agent)
        ]

        return({'id': task_id,
                'name': task_name,
                'details': task_details,
                'pointers': task_pointers,
                'stage_list': task_stage_list})


class StageDef(object): pass