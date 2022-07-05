import rospy
from rospy import Subscriber, Service, Publisher
from rasberry_coordination.srv import QueryBattery, QueryBatteryResponse
from rasberry_coordination.msg import AgentRegistrationList, AgentRegistration, AgentStateList, AgentState

class HealthService(object):

    def __init__(self, agent_details):
        self.agent_details = agent_details

        # 
        self.agent_registration = Publisher('/rasberry_coordination/monitoring/agent_registrations', AgentRegistrationList, latched=True, queue_size=2)
        self.agent_states = Publisher('/rasberry_coordination/monitoring/agent_states', AgentStateList, latched=True, queue_size=2)

        # Start Services
        self.battery_service = Service('/rasberry_coordination/monitoring/battery', QueryBattery, self.check_battery)


    def publish_registrations(self):
        try:
            lst = [AgentRegistration({'agent_id':a.agent_id, 'registered': a.registered()}) for a in self.agent_details.values()]
            self.agent_registration.publish(AgentRegistrationList({'list':lst}))
        except:
            pass

    def publish_states(self):
        try:
            lst = [AgentState({'agent_id': a.agent_id, 'current_task_id': a['id'], 'current_task': ['name'], 'stage': type(a()), 'details': a['details']}) for a in self.agent_details.values()]
            self.agent_states.publish(AgentStateList({'agents':lst}))
        except:
            pass

    def check_battery(self, msg):
        agent = self.agent_details[msg.agent_id]
        return agent.modules['health_monitoring'].interface.battery_level

