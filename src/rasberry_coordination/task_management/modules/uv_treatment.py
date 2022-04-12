"""UV Treatment"""

from copy import deepcopy
from pprint import pprint
from rospy import Time, Duration, Subscriber, Publisher, Time

from std_msgs.msg import String as Str
from rasberry_coordination.msg import TopoLocation

from rasberry_coordination.actions.action_manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.task_management.base import TaskDef as TDef, StageDef as SDef, InterfaceDef as IDef

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class InterfaceDef(object):

    class phototherapist(object):
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\":\"%s\"}' % (self.agent.agent_id, state))
            logmsg(category="COMMS", msg="Publishing: (%s)" % msg)

        def __init__(self, agent):
            self.agent = agent

            self.light_status = False

            self.sub_edge     = Subscriber('/%s/uv_treatment/initiate_task/edge'     % agent.agent_id, TopoLocation, self.edge)
            self.sub_row      = Subscriber('/%s/uv_treatment/initiate_task/row'      % agent.agent_id, TopoLocation, self.row)
            self.sub_tunnel   = Subscriber('/%s/uv_treatment/initiate_task/tunnel'   % agent.agent_id, TopoLocation, self.tunnel)
            # self.sub_schedule = Subscriber('/%s/initiate_task/schedule' % agent.agent_id, Str, self.schedule)

        def edge(self, msg):
            if self.agent.registration:
                # msg.type = "tall"
                # msg.tunnel = 1
                # msg.row = 3
                # msg.edge_nodes = [0,1]
                nodeA = "%s-t%s-r%s-c%s"%(msg.type, msg.tunnel, msg.row, msg.edge_node[0])
                nodeB = "%s-t%s-r%s-c%s"%(msg.type, msg.tunnel, msg.row, msg.edge_node[1])

                logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat edge")
                self.agent.add_task(task_name='uv_treatment_treat_edge', contacts={'row_ends': [nodeA, nodeB]})
        def row(self, msg):
            if self.agent.registration:
                # msg.type = "tall"
                # msg.tunnel = 1
                # msg.row = 3
                row = "%s-t%s-r%s"%(msg.type, msg.tunnel, msg.row)

                logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='uv_treatment_treat_row', details={"row": row})
        def tunnel(self, msg):
            if self.agent.registration:
                #msg.type = "tall"
                #msg.tunnel = 1
                tunnel = "%s-t%s"%(msg.type, msg.tunnel)

                logmsg(category="UVTask", id=self.agent.agent_id, msg="Request to treat row")
                self.agent.add_task(task_name='uv_treatment_treat_tunnel', details={"tunnel": tunnel})

        def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
        def __setitem__(self, key, val): self.__setattr__(key, val)


    class controller(IDef.RasberryInterfacing_ProtocolManager):
        def sar_BEGUN(self):
            task_scope, details = self.get_task('uv_treatment')
            task_name = 'send_uv_treatment'
            if task_name and details: self.agent.add_task(task_name=task_name, details=details)
        def sar_CANCEL(self):
            if self.agent['name'] == 'send_uv_treatment':
                logmsg(level="error", category="IDef", id=self.agent.agent_id, msg="has task")
                self.agent.set_interrupt('reset', 'uv_treatment', self.agent['id'], "Task")
        def sar_EMERGENCY_STOP(self):
            if self.agent['name'] == 'send_uv_treatment':
                self.agent.set_interrupt('pause', 'uv_treatment', self.agent['id'], "Task")
                if 'phototherapist' in self.agent['contacts'] and 'Pause' not in self.agent['contacts']['phototherapist']().get_class():
                    self.agent['contacts']['phototherapist'].set_interrupt('pause', 'uv_treatment', self.agent['id'], "Task")
        def sar_EMERGENCY_RESUME(self):
            if self.agent['name'] == 'send_uv_treatment':
                self.agent.set_interrupt('resume', 'uv_treatment', self.agent['id'], "Task")
                if 'phototherapist' in self.agent['contacts'] and 'Pause' in self.agent['contacts']['phototherapist']().get_class():
                    self.agent['contacts']['phototherapist'].set_interrupt('resume', 'uv_treatment', self.agent['id'], "Task")


class TaskDef(object):
    """ Constructors for UV Tasks """

    """ Tasks """
    @classmethod
    def uv_treatment_treat_edge(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='uv_treatment',
                     name="uv_treatment_treat_edge",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindStartNode(agent),
                         StageDef.NavigateToUVStartNode(agent),
                         StageDef.EnableUVLight(agent),
                         StageDef.NavigateToUVEndNode(agent),
                         StageDef.DisableUVLight(agent)
                     ]))
    @classmethod
    def uv_treatment_treat_row(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='uv_treatment',
                     name="uv_treatment_treat_row",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         SDef.FindRowEnds(agent, details['row']),
                         SDef.FindStartNode(agent),
                         StageDef.NavigateToUVStartNode(agent),
                         StageDef.EnableUVLight(agent),
                         StageDef.NavigateToUVEndNode(agent),
                         StageDef.DisableUVLight(agent)
                     ]))
    @classmethod
    def uv_treatment_treat_tunnel(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='uv_treatment',
                     name="uv_treatment_treat_tunnel",
                     details=details,
                     contacts=contacts,
                     initiator_id=initiator_id,
                     responder_id="",
                     stage_list=[
                         SDef.StartTask(agent, task_id),
                         StageDef.FindRowsUV(agent, details['tunnel'])
                     ]))

    """ Control from SAR """
    @classmethod
    def send_uv_treatment(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='uv_treatment',
                    name="send_uv_treatment",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        SDef.StartTask(agent, task_id),
                        StageDef.AssignPhototherapist(agent, details),
                        StageDef.AwaitCompletion(agent),
                    ]))


class StageDef(object):

    class FindRowsUV(SDef.FindRows):
        """Used to assign the uv_treatment_treat_row task to all rows in the given tunnel."""
        def __init__(self, agent, tunnel):
            """Call super to set uv_treatment_treat_row as task to apply"""
            super(StageDef.FindRowsUV, self).__init__(agent, tunnel, 'uv_treatment_treat_row')

    class NavigateToUVStartNode(SDef.NavigateToNode):
        """Used to navigate to a given start node"""
        def __init__(self, agent):
            """Call to super to set the navigation target as the node stored in the action association"""
            super(StageDef.NavigateToUVStartNode, self).__init__(agent, association='start_node')
        def _start(self):
            if 'controller' in self.agent['contacts']:
                self.agent['contacts']['controller'].modules['uv_treatment'].interface.notify("sar_AWAIT_START")
            super(StageDef.NavigateToUVStartNode, self)._start()

    class NavigateToUVEndNode(SDef.NavigateToNode):
        """Used to navigate to a given end node"""
        def __init__(self, agent):
            """Call to super to set the navigation target as the node stored in the action association"""
            super(StageDef.NavigateToUVEndNode, self).__init__(agent, association='end_node')

    class EnableUVLight(SDef.NotifyTrigger):
        """Used to enable the UV light on the robot"""
        def __init__(self, agent):
            """Call to initialise a light_status message of ENABLE_LIGHT to send on start and set rviz robot to blue"""
            super(StageDef.EnableUVLight, self).__init__(agent, trigger='light_status', msg="ENABLE_LIGHT", colour='blue')
        def _end(self):
            if 'controller' in self.agent['contacts']:
                self.agent['contacts']['controller'].modules['uv_treatment'].interface.notify("sar_AWAIT_TASK_COMPLETION")

    class DisableUVLight(SDef.NotifyTrigger):
        """Used to disable the UV light on the robot"""
        def __init__(self, agent):
            """Call to initialise a light_status message of DISABLE_LIGHT to send on start and set rviz robot to clear"""
            super(StageDef.DisableUVLight, self).__init__(agent, trigger='light_status', msg="DISABLE_LIGHT", colour='')
        def _end(self):
            if 'controller' in self.agent['contacts']:
                if len([s for s in self.agent['stage_list'][:-1] if 'DisableUVLight' in s.get_class()]) == 0:
                    # if there is no more stages in stagslit, set flag on controller?
                    self.agent['contacts']['controller']['phototherapist_completion_flag'] = True

    class AssignPhototherapist(SDef.ActionResponse):
        """Used to identify the closest phototherapist."""
        def __init__(self, agent, details):
            """ Mark the details of the associated Action """
            print(details)
            self.details = details
            self.response_task = 'uv_treatment_treat_'+details['scope']
            self.contacts = {'controller': agent}
            if details['scope'] == "edge": self.contacts['row_ends'] = details['nodes']
            if details['robot'] != "closest": self.action['list'] = [details['robot']]
            super(StageDef.AssignPhototherapist, self).__init__(agent)
            self.action = ActionDetails(type='search', grouping='agent_descriptor', descriptor='phototherapist', style='closest_agent')
            self.contact = 'phototherapist'
        def _end(self):
            super(StageDef.AssignPhototherapist, self)._end()
            self.agent.modules['uv_treatment'].interface.notify("sar_AWAIT_START")
            self.agent['contacts']['phototherapist'].add_task(task_name=self.response_task,
                                                              task_id=self.agent['id'],
                                                              details=self.details,
                                                              contacts=self.contacts,
                                                              initiator_id=self.agent.agent_id)



    class AwaitCompletion(SDef.Idle):
        def _start(self):
            super(StageDef.AwaitCompletion,self)._start()
            self.agent['phototherapist_completion_flag'] = False
        def _query(self):
            success_conditions = [self.agent['phototherapist_completion_flag']]
            self.flag(any(success_conditions))
        def _end(self):
            self.agent.modules['uv_treatment'].interface.notify("sar_COMPLETE")






    """
    
    assignAgent needs ability to assign by name
    
    publish configs to sar

    """