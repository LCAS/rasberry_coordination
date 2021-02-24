#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------


from abc import ABCMeta, abstractmethod
from rospy import Subscriber, Publisher, Time
from std_msgs.msg import String as Str
from rasberry_coordination.msg import KeyValuePair
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.robot import Robot as RobotInterface_Old
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef


""" Agent Interface Devices / Systems """
class AgentInterface(object):
    def __init__(self, agent_id, responses, sub_topic, pub_topic):
        self.sub = Subscriber(sub_topic, Str, self.callback)
        self.pub = Publisher(pub_topic, Str, queue_size=5)
        self.agent_id = agent_id
        self.responses = responses

    def callback(self, msg): #Look into sub/feature
        msg = eval(msg.data)

        if "states" in msg: #car callback sends two msgs
            return

        if msg['user'] == self.agent_id:
            if msg['state'] in self.responses:
                self.responses[msg['state']]()
    def notify(self, state):
        msg = Str('{\"user\":\"%s\", \"state\": \"%s\"}' % (self.agent_id, state))
        logmsg(msg="PUBLISHING \"%s\"" % msg)
        self.pub.publish(msg)

#Definitions for CallARobot/LoadARobot (picker), UnloadARobot (storage), and Robot_Interface (robot)
class CAR_App(AgentInterface):
    def __init__(self, agent_id, responses, sub_topic='/car_client/get_states', pub_topic='/car_client/set_states'):
        super(CAR_App, self).__init__(agent_id, responses, sub_topic, pub_topic)
class LAR_Device(AgentInterface):
    def __init__(self, agent_id, responses, sub_topic='/lar/get_states', pub_topic='/lar/set_states'):
        super(LAR_Device, self).__init__(agent_id, responses, sub_topic, pub_topic)
class UAR_Device(AgentInterface):
    def __init__(self, agent_id, responses, sub_topic='/uar/get_states', pub_topic='/uar/set_states'):
        super(UAR_Device, self).__init__(agent_id, responses, sub_topic, pub_topic)
class Robot_Interface(AgentInterface):
    def __init__(self, agent_id, responses, sub_topic='/robot/get_states', pub_topic='/robot/set_states'):
        super(Robot_Interface, self).__init__(agent_id, responses, sub_topic, pub_topic)
#TODO \/
class CAR_Device(AgentInterface):
    pass
class LAR_App(LAR_Device):
    pass
class UAR_App(UAR_Device):
    pass
