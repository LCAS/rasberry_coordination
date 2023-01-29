"""Health Monitoring"""

from copy import deepcopy

from std_msgs.msg import String as Str, Bool, Float32
from thorvald_base.msg import BatteryArray as Battery, ControllerArray
from polytunnel_navigation_actions.msg import RowTraversalHealth

from rospy import Time, Duration, Subscriber, Publisher, Time

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.task_management.containers.Task import TaskObj as Task
from rasberry_coordination.task_management.__init__ import Stages, Interfaces

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class RobotDebug(Interface):

    def __init__(self, agent, details):
        super(RobotDebug, self).__init__(agent, details)
        self.speaker = self.agent.speaker

        self.motors_have_scripts = True
        self.motors_turned_on = True
        self.agent.local_properties['battery_level'] = 50.0

        self.in_auto_mode = True
        self.row_trav_paused = False


    def idle(self, task_id=None, details=None, contacts=None, initiator_id=""):
        # Low battery is added here as new task once idle
        # Critical battery is forced into next task when identified
        if self.agent.modules['rasberry_health_monitoring_pkg'].interface.battery_low():
            return self.charge_at_charging_station(task_id=task_id, details=details, contacts=contacts)


    def charge_at_charging_station(self, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='rasberry_health_monitoring_pkg',
                    name="charge_at_charging_station",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['rasberry_health_monitoring_pkg']['StartChargeTask'](self.agent),
                        Stages['assignment']['AssignNode'](self.agent, contact_id='reserved_charging_station', node_descriptor='charging_station'),
                        Stages['rasberry_health_monitoring_pkg']['NavigateToChargeNode'](self.agent, contact_id='reserved_charging_station'),
                        Stages['rasberry_health_monitoring_pkg']['Charge'](self.agent)
                    ]))

    def send_for_mot(self, task_id=None, details=None, contacts=None, initiator_id=""):
        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduled request to have MOT")

        msg = details['msg']
        target = msg.criteria.nodes[0]
        duration = 150

        return(Task(id=task_id,
                    module='rasberry_health_monitoring_pkg',
                    name="send_to_mot",
                    details=details,
                    contacts=contacts,
                    initiator_id=self.agent.agent_id,
                    responder_id="",
                    stage_list=[
                        Stages['base']['StartTask'](self.agent),
                        Stages['navigation']['NavigateToNode'](self.agent, target=target),
                        Stages['base']['Timeout'](self.agent, duration=duration)
                    ]))


    def enable_navigation(self):
        print('force replanning attempt')
        if (self.in_auto_mode) and (self.motors_turned_on) and (self.motors_have_scripts):
            print('replanning forced by health_monitoring')
            self.agent.cb['force_replan']()
        pass

    def is_idle(self):
        return self.agent().accepting_new_tasks

    def is_charging(self):
        return self.agent['task_name'] is "charge_at_charging_station"

    def has_charging_task(self):
        return "charge_at_charging_station" in [t.name for t in self.agent.task_buffer]+[self.agent['task_name']]

    def battery_critical(self):
        return False

    def battery_low(self):
        return False

