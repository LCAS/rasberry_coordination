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


class Robot(Interface):

    def __init__(self, agent, details):
        super(Robot, self).__init__(agent, details)
        self.speaker = self.agent.speaker

        self.motors_have_scripts = True
        self.motor_issues_sub = Subscriber('/%s/health_monitoring/motor_issues' % (self.agent.agent_id), Bool, self.motor_issues_cb)

        self.motors_turned_on = False
        self.motors_turned_on_sub = Subscriber('/%s/health_monitoring/motors_turned_on' % (self.agent.agent_id), Bool, self.motors_turned_on_cb)

        self.agent.local_properties['battery_level'] = None
        self.battery_estimate_sub = Subscriber('/%s/health_monitoring/battery_estimate' % (self.agent.agent_id), Float32, self.battery_estimate_cb)

        self.in_auto_mode = None
        self.auto_mode_sub = Subscriber('/%s/debug/auto_mode' % (self.agent.agent_id), Bool, self.auto_mode_cb)

        self.row_trav_paused = None
        self.row_trav_sub = Subscriber('/%s/health_monitoring/row_traversal' % (self.agent.agent_id), RowTraversalHealth, self.row_trav_cb)



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
        LP = self.agent.local_properties
        CRIT = fetch_property('health_monitoring', 'critical_battery_limit')
        if 'battery_level' in LP and LP['battery_level'] < CRIT: return True

    def battery_low(self):
        LP = self.agent.local_properties
        MIN = fetch_property('health_monitoring', 'low_battery_limit')
        CRIT = fetch_property('health_monitoring', 'critical_battery_limit')
        if 'battery_level' in LP and CRIT < LP['battery_level'] <= MIN: return True

    def motor_issues_cb(self, motor_status):
        if self.motors_have_scripts != motor_status.data: return
        self.motors_have_scripts = not motor_status.data
        if self.motors_have_scripts:
            logmsg(level="warn", category="HEALTH", id=self.agent.agent_id, msg="Agent has no missing scripts.")
            self.enable_navigation()
        else:
            logmsg(level="warn", category="HEALTH", id=self.agent.agent_id, msg="Agent has missing scripts.")

    def motors_turned_on_cb(self, motors_turned_on):
        if self.motors_turned_on == motors_turned_on.data: return
        self.motors_turned_on = motors_turned_on.data
        if self.motors_turned_on:
            logmsg(level="warn", category="HEALTH", id=self.agent.agent_id, msg="Agent motors are turned ON.")
            self.speaker("motors: on")
            self.enable_navigation()
        else:
            logmsg(level="warn", category="HEALTH", id=self.agent.agent_id, msg="Agent motors are turned OFF")
            self.speaker("motors: off")


    def battery_estimate_cb(self, battery_estimate):
        # Save the data to the agent object
        self.agent.local_properties['battery_level'] = battery_estimate.data

        # Check if motors are turned on, if not we shouldnt monitor battery output
        if not self.motors_turned_on: return

        # Add charging task if battery is critical, and agent isnt charging
        if self.battery_critical() and not self.is_charging():
            logmsg(level="error", category="HEALTH", id=self.agent.agent_id, msg="Battery crit check: %s"%self.agent.local_properties['battery_level'])
            self.agent.task_buffer = [t for t in self.agent.task_buffer if t.name != "charge_at_charging_station"]  # Filter charging task from buffer
            self.agent.add_task(module="rasberry_health_monitoring_pkg", name="charge_at_charging_station", index=0)
            self.speaker("battery: critical")


        # Add charging task if battery is low, agent isnt planning to charge, and stage is idle
        if self.battery_low() and not self.has_charging_task() and self.is_idle():
            logmsg(level="error", category="HEALTH", id=self.agent.agent_id, msg="Battery low check: %s"%self.agent.local_properties['battery_level'])
            self.agent.add_task(module="rasberry_health_monitoring_pkg", name="charge_at_charging_station")
            self.speaker("battery: low")

        # TODO:
        # we need to add an extra condition here to check again after 5 seconds of low/critical battery to see if it is still passes the conditions


    def auto_mode_cb(self, in_auto_mode):
        if self.in_auto_mode == in_auto_mode.data: return
        self.in_auto_mode = in_auto_mode.data
        if self.in_auto_mode:
            logmsg(level="warn", category="HEALTH", id=self.agent.agent_id, msg="Agent is in AUTONOMOUS mode.")
            self.speaker("autonomy: enabled")
            self.enable_navigation()
        else:
            logmsg(level="warn", category="HEALTH", id=self.agent.agent_id, msg="Agent is in MANUAL mode.")
            self.speaker("autonomy: disabled")

    def row_trav_cb(self, msg):
        if self.row_trav_paused == msg.paused.data: return
        self.row_trav_paused = msg.paused.data
        if self.row_trav_paused:
            logmsg(level="warn", category="HEALTH", id=self.agent.agent_id, msg="Row Traversal is Paused.")
            self.speaker("row traversal: paused")
        else:
            logmsg(level="warn", category="HEALTH", id=self.agent.agent_id, msg="Row Traverlsal is Active")
            self.speaker("row traversal: active")
            self.enable_navigation()


        def _end(self):
            """Enable registration once task is ended"""
            self.agent.registration = True
