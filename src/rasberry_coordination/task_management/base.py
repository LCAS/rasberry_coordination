"""Base"""

from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, get_param
from std_msgs.msg import Bool, String as Str
from diagnostic_msgs.msg import KeyValue
import strands_executive_msgs.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.actions.action_manager import ActionDetails
from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.encapsuators import TaskObj as Task, LocationObj as Location
from rasberry_coordination.robot import Robot, VirtualRobot
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch

try: from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef, fetch_property
except: pass


class InterfaceDef(object):

    class TOC_Interface(object):
        def __init__(self, coordinator):
            logmsg(category="SETUP", msg="TOC Initialised")
            self.coordinator = coordinator
            ns = "/rasberry_coordination"

            """ TOC Publishers """
            self.previous_task_list = None
            self.previous_task_list_2 = None
            self.active_tasks_pub = Publisher('%s/active_tasks_details'%ns, TasksDetailsList, latch=True, queue_size=5)
            self.task_pause_pub = Publisher('%s/pause_state'%ns, Bool, queue_size=5, latch=True)

            """ TOC Dynamic Task Management """
            Subscriber('/rasberry_coordination/dtm', Interruption, self.InterruptTask)

            """ Reset the TOC Active Task List """
            self.ResetTaskList()

        """ Short-Definition Convenience Functions """
        def ResetTaskList(self):
            t = TasksDetailsList()
            task = SingleTaskDetails()
            task.task_id = "__RESET__"
            t.tasks.append(task)
            self.publish_task_list(t)
        def UpdateTaskList(self):
            task_list = self.generate_active_tasks_list()
            if self.previous_task_list != task_list and task_list.tasks:
                self.previous_task_list = task_list
                self.publish_task_list(task_list)
                return True
        def EndTask(self, E):
            task_list = self.generate_active_tasks_list()
            task_list.tasks = [t for t in task_list.tasks if t.task_id not in E]
            [task_list.tasks.append(self.generate_completed_task(task_id)) for task_id in E if task_id]
            self.publish_task_list(task_list)

        """ Active Task List Modifers """
        def generate_active_tasks_list(self):
            """ Publish updated list of Active Tasks to TOC """
            task_list = TasksDetailsList()
            for agent in self.coordinator.get_agents():

                # If a task exists
                if agent['id']:

                    # If task is already added, move on
                    if agent['id'] in [T.task_id for T in task_list.tasks]: continue

                    # Get task details
                    task = SingleTaskDetails()
                    task.task_id = agent['id']
                    task.initiator_id = agent['initiator_id']
                    task.responder_id = agent['responder_id']

                    # Get state of task from initiator
                    i = agent['initiator_id'] or agent.agent_id
                    init = self.coordinator.agent_manager.agent_details[i]
                    stage_repr = init().__repr__()
                    if '(' in stage_repr:
                        stage_content = "(" + stage_repr.split('(')[-1]
                        stage_cls = stage_repr.split('(')[0].split('.')[-1]
                    else:
                        stage_content = ""
                        stage_cls = stage_repr.split('.')[-1]
                    task.state = stage_cls + stage_content.replace('()','')

                    # Add task to list
                    task_list.tasks.append(task)
            return task_list
        def generate_completed_task(self, task_id=None):
            task = SingleTaskDetails()
            task.task_id = task_id
            task.state = "CANCELLED"
            return task
        def publish_task_list(self, task_list):
            logmsg(level="info", category="SECT", id="SECTION", msg="\033[01;04;92mTOC\033[38;5;231m\033[0m")
            logmsg(category="TOC", msg="Active Tasks:")
            [logmsg(category="TOC", msg="    | %s\t  -- %s [%s,%s]" % (t.task_id, t.state.replace("Idle", "\033[01;32mIdle\033[0m"), t.initiator_id, t.responder_id)) for t in task_list.tasks]
            # logmsg(category="null")
            self.active_tasks_pub.publish(task_list)

        """ Dynamic Task Management """
        def InterruptTask(self, m):
            #For targets, set interruption to [toc_cancel|toc_pause|toc_unpause]
            logmsg(category="null")
            logmsg(category="DTM", id="toc", msg="Interruption made on TOC channels of type: %s" % m.interrupt)
            A = {a.agent_id:a for a in self.coordinator.get_agents()}

            if m.scope in [0, "Coord", "Coordinator"]:
                # Modify all tasks
                logmsg(category="DTM", msg="    - to affect all agents.")
                # [a.set_interrupt(m.interrupt, a.module, a['task_id'], m.scope, quiet=True) for a in A.values() if a['task_id']]

                if m.interrupt == "reset":
                    for a in A.values():
                        if a['task_id'] and a.agent_id == a['initiator_id']:
                            logmsg(category="DTM", msg="      | release")
                            a.set_interrupt("reset", a.module, a['task_id'], m.scope, quiet=True)
                else:
                    [a.set_interrupt(m.interrupt, a.module, a['task_id'], m.scope, quiet=True) for a in A.values() if a['task_id']]


            elif m.scope in [1, "Task"]:
                # Modify all agents on specific task
                logmsg(category="DTM", msg="    - to affect task: %s." % m.target)
                # [a.set_interrupt(m.interrupt, a.module, a['task_id'], m.scope, quiet=True) for a in A.values() if a['task_id'] and a['task_id'] == m.target]

                if m.interrupt == "reset":
                    for a in A.values():
                        if (a['task_id']) and (a['task_id'] == m.target) and (a.agent_id == a['initiator_id']):
                            logmsg(category="DTM", msg="      | release")
                            a.set_interrupt("reset", a.module, a['task_id'], m.scope, quiet=True)
                else:
                    [a.set_interrupt(m.interrupt, a.module, a['task_id'], m.scope, quiet=True) for a in A.values() if a['task_id'] and a['task_id'] == m.target]



            elif m.scope in [2, "Agent"]:
                # Modify specific agent's task
                logmsg(category="DTM", msg="    - to affect agent: %s." % m.target)
                A[m.target].set_interrupt(m.interrupt, A[m.target].module, A[m.target]['task_id'], m.scope, quiet=True)

            else:
                print(m)


    class AgentInterface(object):
        def __init__(self, agent, responses, sub, pub):
            self.agent = agent
            self.responses = responses
            self.pub = Publisher(pub, Str, queue_size=5)
            self.sub = Subscriber(sub, Str, self.callback, agent.agent_id)
        def callback(self, msg, agent_id):  # Look into subscribing to a /topic/feature
            msg = eval(msg.data)
            if "states" in msg: return # car callback sends two msgs, this filters second #TODO: remove this
            if msg['user'] == agent_id:
                if msg['state'] in self.responses:
                    self.responses[msg['state']]()
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\":\"%s\"}' % (self.agent.agent_id, state))
            logmsg(category="COMMS", msg="        - Publishing: (%s)" % msg)
            self.pub.publish(msg)


    class RasberryInterfacing_ProtocolManager(object):
        def __init__(self, agent):
            self.agent = agent
            self.msg = None
            self.pub = Publisher('/car_client/set_states_kv', KeyValue, queue_size=5)
            self.sub = Subscriber('/car_client/get_states_kv', KeyValue, self.callback, agent.agent_id)
            self.loc_pub = Publisher('/car_client/set_closest_node_kv', KeyValue, queue_size=5)
            self.loc_sub = Subscriber('%s/closest_node'%self.agent.agent_id, Str, self.loc_cb)
            self.notify("CONNECTED")
        def loc_cb(self, msg):
            self.loc_pub.publish(KeyValue(key=self.agent.agent_id, value=msg.data))
        def callback(self, msg, agent_id):
            if msg.key == agent_id:
                state = msg.value.split('-')[0]
                if state in dir(self):
                    logmsg(category="IDef", id=agent_id, msg="State changed to: %s" % state)
                    self.msg = msg
                    getattr(self, state)()

        def notify(self, state):
            msg = KeyValue(key=self.agent.agent_id, value=state)
            logmsg(category="IDef", msg="        - Publishing: (%s)" % str(msg).replace('\n',' | '))
            self.pub.publish(msg)

        def get_task(self, module):
            state, row, edge, task, robot = self.msg.value.split('-')
            nodes = []
            if task != module: return (None,None)
            elif edge != "all":
                task_scope = 'edge'
                nodes = ["r%s-c%s" % (row, e) for e in edge.split('>')]
            elif row != "all":
                task_scope = 'row'

            return (task_scope, {'row': 'r'+row,
                                 'edge': edge,
                                 'nodes': nodes,
                                 'robot': robot,
                                 'scope': task_scope})


    class robot(object):
        def __init__(self, agent, Type):
            self.agent = agent
            self.agent.navigation_interface = Type
            self.disconnect_sub = Subscriber('/%s/base/disconnect' % agent.agent_id, Str, self.disconnect)
            self.move_idle_sub = Subscriber('/%s/base/move_idle' % agent.agent_id, Str, self.move_idle)
        def disconnect(self, msg):
            logmsg(category="Task", id=self.agent.agent_id, msg="Request to disconnect")
            node_id = msg.data or self.agent.goal or self.agent.location(accurate=True)
            self.agent.add_task(task_name='exit_at_node', contacts={"exit_node":node_id}, index=0)
        def move_idle(self, msg):
            logmsg(category="Task", id=self.agent.agent_id, msg="Request to Move while Idle")

            # if idle:
            if not isinstance(self.agent(), StageDef.Idle):
                logmsg(category="Task", msg="    - agent not idle")
                return

            # if node valid:
            if not self.agent.map_handler.is_node(msg.data):
                logmsg(category="Task", msg="    - node is invalid")
                return

            # add task
            self.agent.add_task(task_name='move_idle', contacts={"target":msg.data})

    class base_robot(robot):
        def __init__(self, agent):
            super(InterfaceDef.base_robot, self).__init__(agent, Robot(agent.agent_id, agent.speaker))
    class base_virtual_robot(robot):
        def __init__(self, agent):
            sd = fetch_property('base', 'virtual_robot_step_delay')
            VR = VirtualRobot(agent.agent_id, agent, step_delay=sd)
            super(InterfaceDef.base_virtual_robot, self).__init__(agent, VR)


    class base_human(object):
        def __init__(self, agent):
            self.agent = agent
    class base_localised_human(base_human):
        pass


class TaskDef(object):

    """ Runtime Method for Init Task Definitions """
    @classmethod
    def base_robot_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='base',
                     name="base_robot_init",
                     details=details,
                     contacts=contacts,
                     initiator_id=agent.agent_id,
                     responder_id="",
                     stage_list=[
                         StageDef.StartTask(agent, task_id),
                         StageDef.SetUnregister(agent),
                         StageDef.WaitForMap(agent),
                         StageDef.WaitForLocalisation(agent),
                         StageDef.SetRegister(agent)
                     ]))

    @classmethod
    def base_virtual_robot_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return (Task(id=task_id,
                     module='base',
                     name="base_virtual_robot_init",
                     details=details,
                     contacts=contacts,
                     initiator_id=agent.agent_id,
                     responder_id="",
                     stage_list=[
                         StageDef.StartTask(agent, task_id),
                         StageDef.SetUnregister(agent),
                         StageDef.WaitForMap(agent),
                         StageDef.EnableVirtualLocalisation(agent),
                         StageDef.WaitForLocalisation(agent),
                         StageDef.SetRegister(agent)
                     ]))

    @classmethod
    def base_human_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        agent.current_node = agent.cb['default_location']
        return(Task(id=task_id,
                    module='base',
                    name="base_human_init",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.SetUnregister(agent),
                        StageDef.WaitForMap(agent),
                        StageDef.SendInfo(agent),
                        StageDef.SetRegister(agent)
                    ]))
    @classmethod
    def base_localised_human_init(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="base_human_init",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.SetUnregister(agent),
                        StageDef.WaitForMap(agent),
                        StageDef.WaitForLocalisation(agent),
                        StageDef.SendInfo(agent),
                        StageDef.SetRegister(agent)
                    ]))

    """ Idle Tasks """
    @classmethod
    def base_robot_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return TaskDef.wait_at_base(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def base_virtual_robot_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return TaskDef.wait_at_base(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def base_human_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return TaskDef.idle(agent=agent, task_id=task_id, details=details, contacts=contacts)
    @classmethod
    def base_localised_human_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return TaskDef.idle(agent=agent, task_id=task_id, details=details, contacts=contacts)

    """ Runtime Method for Idle Task Definitions """
    @classmethod
    def idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="idle",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.Idle(agent)
                    ]))
    @classmethod
    def wait_at_base(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="wait_at_base",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        # StageDef.Exit(agent)
                        StageDef.AssignBaseNodeIdle(agent),
                        StageDef.NavigateToBaseNodeIdle(agent),
                        StageDef.Idle(agent)
                    ]))
    @classmethod
    def move_idle(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="move_idle",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.StartTask(agent, task_id),
                        StageDef.NavigateToTargetNode(agent),
                        StageDef.Idle(agent)
                    ]))
    @classmethod
    def exit_at_node(cls, agent, task_id=None, details=None, contacts=None, initiator_id=""):
        return(Task(id=task_id,
                    module='base',
                    name="exit_at_node",
                    details=details,
                    contacts=contacts,
                    initiator_id=agent.agent_id,
                    responder_id="",
                    stage_list=[
                        StageDef.SetUnregister(agent),
                        StageDef.NavigateToExitNode(agent),
                        StageDef.Exit(agent)
                    ]))

    """ Dynamic Task Management """
    @classmethod
    def release_task(cls, agent):
        logmsg(category="DTM", msg="    | releasing task %s" % (agent['name']))
        agent.task = None
    @classmethod
    def restart_task(cls, agent):
        logmsg(category="DTM", msg="    | restarting task %s" % (agent['name']))
        agent.add_task(task_name=agent['name'], task_id=agent['id'], index=0, quiet=True)
        agent.task = None


class StageDef(object):

    class StageBase(object):
        """Base class for all Stages"""
        def get_class(self):
            """Cleaned class name for explicit class-type queries"""
            return str(self.__class__)\
                .replace("<class 'rasberry_coordination.task_management.","")\
                .replace("'>","")\
                .replace("modules.","")
        def __repr__(self):
            """Simplified representation of class for clean informative logging"""
            return self.get_class()
        def suspend(self):
            """Collection function for all actions to call when a stage is suspended"""
            self.new_stage = True
        def flag(self, completion):
            """Collection function when a stage is to be flagged as completed"""
            # TODO: this is not very good here, the state of an agent being interrupted should be handled abstractly
            if not self.agent.interruption:
                self.stage_complete = completion
        def __init__(self, agent):
            """Class initialisation for populating default values"""
            self.agent = agent
            self.action_required = False
            self.route_required = False  #True: trigger a route search
            self.route_found = False  #True: trigger a route publish
            self.stage_complete = False
            self.new_stage = True
            self.target = None
            self.target_agent = None
            self.action = None
            self.accepting_new_tasks = False
            self.summaries = {}
        def _start(self):
            """Stage start, called when this is the active stage."""
            logmsg(category="stage", id=self.agent.agent_id, msg="Begun stage %s" % self)
            self.start_time = Time.now()
            self.accepting_new_tasks = False
        def _query(self):
            """Used to define the criteria which ust be met for the stage to be completed"""
            success_conditions = []  # What should be queried to tell if the stage is completed?
            self.flag(any(success_conditions))
        def _end(self):
            """Used to set any fields as the stage is about to be removed"""
            self.stage_complete = False
            pass
    class StartTask(StageBase):
        """Called as the first stage of every task, to generate and apply a task_id."""
        def __init__(self, agent, task_id=None):
            """Generate a unique Task ID of {agent_id}_{total_tasks++}."""
            super(StageDef.StartTask, self).__init__(agent)
            self.task_id = task_id if task_id else "%s_%s" % (self.agent.simple_agent_id(), self.agent.total_tasks)
            self.agent.total_tasks += 1
        def _start(self):
            """Being at the head of the task, once the task is begun the task_id is adopted as the active task_id."""
            self.agent.task_details = {}
            super(StageDef.StartTask, self)._start()
            self.agent['id'] = self.task_id #Set task_id as active_task_id for agent
            self.agent['start_time'] = Time.now()
            self.agent.cb['format_agent_marker'](self.agent, style='')
        def _query(self):
            """Complete the stage without any condition"""
            self.flag(True)
    class ActionResponse(StageBase):
        def __init__(self, agent):
            """Enable action"""
            super(StageDef.ActionResponse, self).__init__(agent)
            self.action_required = True
            self.contact = None
        def _query(self):
            """Complete once action has generated a result"""
            success_conditions = [self.action.response != None]
            self.flag(any(success_conditions))
        def _end(self, contact_type='responder_id'):
            """Save action response to contacts"""
            super(StageDef.ActionResponse, self)._end()
            resp = self.action.response
            if self.contact:
                if '_agent' in str(self.action.style): #TODO: TypeError: argument of type 'NoneType' is not iterable
                    self.agent[contact_type] = self.action.response.agent_id
                self.agent['contacts'][self.contact] = self.action.response


    """ Setup """
    class SetUnregister(StageBase):
        """Called to mark the agent as unregistered"""
        def _start(self):
            """Mark agent as unregistered and send a message to rviz to display the agent as red"""
            super(StageDef.SetUnregister, self)._start()
            self.agent.registration = False
            self.agent.cb['format_agent_marker'](self.agent, style='red')
        def _query(self):
            """Complete the stage without any condition"""
            self.flag(True)
    class WaitForMap(StageBase):
        """Called to suspend task progression till a restricted map is recieved"""
        def _start(self):
            """Begin a subscriber to recieve the tmap"""
            super(StageDef.WaitForMap, self)._start()
            self.agent.map_handler.enable_map_monitoring()
        def _query(self):
            """Complete stage once a tmap is available"""
            success_conditions = [self.agent.map_handler.filtered_node_list]
            self.flag(any(success_conditions))
    class EnableVirtualLocalisation(StageBase):
        """Enable localisation for virtual agents"""
        def _start(self):
            """Enable subscribers to virtual localisation"""
            super(StageDef.EnableVirtualLocalisation, self)._start()
            self.agent.navigation_interface.enable_subscribers()
        def _query(self):
            """Complete the stage without any condition"""
            self.flag(True)
    class WaitForLocalisation(StageBase):
        """Called to suspend task progression till a location for the agent is recieved"""
        def _start(self):
            """Enable location monitoring"""
            super(StageDef.WaitForLocalisation, self)._start()
            self.agent.location.enable_location_monitoring(self.agent.agent_id)
        def _query(self):
            """Complete once location has been identified"""
            success_conditions = [self.agent.location() is not None]
            self.flag(any(success_conditions))
    class SetRegister(StageBase):
        """Called to mark the agent as registered"""
        def _start(self):
            """Mark agent as unregistered and send a message to rviz to display the agent without modified colour"""
            super(StageDef.SetRegister, self)._start()
            self.agent.registration = True
            self.agent.cb['format_agent_marker'](self.agent, style='')
        def _query(self):
            """Complete the stage without any condition"""
            self.flag(True)
        def _end(self):
            super(StageDef.SetRegister, self)._end()
            self.agent.send_car_msg("REGISTERED")

    class SendInfo(ActionResponse):
        """Used to identify the closest available wait_node."""
        def __init__(self, agent):
            """ Mark the details of the associated Action """
            super(StageDef.SendInfo, self).__init__(agent)
            self.action = ActionDetails(type='info', info='send_info')




    """ Idle """
    class Idle(StageBase):
        """Used to suspend activity until the agent has a task."""
        def __repr__(self):
            """Display class with idle location """
            if self.agent: return "%s(%s)" % (self.get_class(), self.agent.location())
            return self.get_class()
        def _start(self):
            super(StageDef.Idle, self)._start()
            self.accepting_new_tasks = True
        def _query(self):
            """Complete once task buffer contains another task, or if battery level is low."""
            success_conditions = [len(self.agent.task_buffer) > 0]
            self.flag(any(success_conditions))

    """ Node identification Stages"""
    class AssignBaseNode(ActionResponse):
        """Used to identify the closest available base_node."""
        def __init__(self, agent):
            """ Mark the details of the associated Action """
            super(StageDef.AssignBaseNode, self).__init__(agent)
            self.action = ActionDetails(type='search', grouping='node_descriptor', descriptor='base_node', style='closest_node')
            self.contact = 'base_node'
    class AssignBaseNodeIdle(AssignBaseNode):
        """Used to identify the closest available base_node."""
        def _start(self):
            super(StageDef.AssignBaseNode, self)._start()
            self.accepting_new_tasks = True
        def _query(self):
            """Complete once action has generated a result"""
            success_conditions = [self.action.response != None,
                                  len(self.agent.task_buffer) > 0]
            self.flag(any(success_conditions))

    class AssignWaitNode(ActionResponse):
        """Used to identify the closest available wait_node."""
        def __init__(self, agent):
            """ Mark the details of the associated Action """
            super(StageDef.AssignWaitNode, self).__init__(agent)
            self.action = ActionDetails(type='search', grouping='node_descriptor', descriptor='wait_node', style='closest_node')
            self.contact = 'wait_node'

    """ Identification of Navigation Targets """
    class FindRowEnds(ActionResponse):
        """Used to identify the two ends of a given row."""
        def __repr__(self):
            """Display the row id to generate tasks for."""
            return "%s(%s)" % (self.get_class(), self.action.descriptor)
        def __init__(self, agent, row):
            """Save the row id of interest"""
            super(StageDef.FindRowEnds, self).__init__(agent)
            self.action = ActionDetails(type='info', info='find_row_ends', descriptor=row)
            self.contact = 'row_ends'
    class FindStartNode(ActionResponse):
        """Used to identify of two nodes, which one is closest ot the agent."""
        def __repr__(self):
            """Display row ends in the repr"""
            if 'row_ends' in self.agent['contacts'] and self.agent['contacts']['row_ends']:
                return "%s(%s)" % (self.get_class(), self.agent['contacts']['row_ends'])
            return self.get_class()
        def _start(self):
            """Define action to find which node of the ends is the closest to start from"""
            super(StageDef.FindStartNode, self)._start()
            lst = self.agent['contacts']['row_ends']
            self.action = ActionDetails(type='search', grouping='node_list', list=lst, style='closest_node')
        def _end(self):
            """Save the closest and furthest node to be the row start and row end"""
            super(StageDef.FindStartNode, self)._end()
            self.agent['contacts']['start_node'] = self.action.response
            self.agent['contacts']['row_ends'].remove(self.action.response)
            self.agent['contacts']['end_node'] = self.agent['contacts']['row_ends'][0]
            logmsg(category="stage", msg="Task to move from %s to %s" % (self.agent['contacts']['start_node'], self.agent['contacts']['end_node']))

    """ Navigation Controllers for Courier """
    class Navigation(StageBase):
        """Base task for all Navigation"""
        def __repr__(self):
            """Display class with idle navigation target """
            # if self.target:
            #     return "%s(%s)"%(self.get_class(), self.target.replace('WayPoint','WP'))
            return "%s" % (self.get_class())
        def __init__(self, agent, association):
            """Identify the location from which the target is identified"""
            super(StageDef.Navigation, self).__init__(agent)
            self.association = association
            self.target = None
        def _start(self):
            """Flag the agent as requirieng a route"""
            super(StageDef.Navigation, self)._start()
            # self.route_found = False  # Has route been identified?
            self.route_required = True  # Has route been published
            # self.state = "" #= "Identified" = "Published"
            logmsg(category="stage", id=self.agent.agent_id, msg="Navigation from %s to %s is begun." % (self.agent.location(accurate=True), self.target))
        def _query(self):
            """Complete when the agents location is identical to the target location."""
            success_conditions = [self.agent.location(accurate=True) == self.target]
            self.flag(any(success_conditions))
        def _end(self):
            """End navigation by refreshing routes for other agents in motion."""
            logmsg(category="stage", id=self.agent.agent_id, msg="Navigation from %s to %s is completed." % (self.agent.location(accurate=True), self.target))
            #self.agent.navigation_interface.cancel_execpolicy_goal()
            #self.target = None
            self.agent.cb['trigger_replan']() #ReplanTrigger
    class NavigateToAgent(Navigation):
        """Used for navigating to a given agent"""
        def _start(self):
            """Start task by setting the target to be a defined agent's current location"""
            self.target = self.agent['contacts'][self.association].location(accurate=True)
            self.target_agent = self.agent['contacts'][self.association]
            super(StageDef.NavigateToAgent, self)._start()
    class NavigateToNode(Navigation):
        """Used for navigating to a given node"""
        def _start(self):
            """Start task by setting the target to a given node"""
            self.target = self.agent['contacts'][self.association]
            super(StageDef.NavigateToNode, self)._start()

    """ Navigation Subclasses """
    class NavigateToBaseNode(NavigateToNode):
        """Used to navigate to a given base_node"""
        def __init__(self, agent):
            """Call super to set association to base_node"""
            super(StageDef.NavigateToBaseNode, self).__init__(agent, association='base_node')
    class NavigateToBaseNodeIdle(NavigateToBaseNode):
        """ Used to Navigate To Base node, but with interruption enabled """
        def _start(self):
            """ enable interuption """
            super(StageDef.NavigateToBaseNodeIdle, self)._start()
            self.accepting_new_tasks = True
        def _query(self):
            """Complete when the agents location is identical to the target location."""
            success_conditions = [self.agent.location(accurate=True) == self.target, 
                                  len(self.agent.task_buffer) > 0]
            self.flag(any(success_conditions))

    class NavigateToExitNode(NavigateToNode):
        """Used to navigate to a given exit_node"""
        def __init__(self, agent):
            """Call super to set association to exit_node"""
            super(StageDef.NavigateToExitNode, self).__init__(agent, association='exit_node')
    class NavigateToWaitNode(NavigateToNode):
        """Used to navigate to a given wait_node"""

        def __init__(self, agent):
            """Call super to set association to wait_node"""
            super(StageDef.NavigateToWaitNode, self).__init__(agent, association='wait_node')

    class NavigateToTargetNode(NavigateToNode):
        """Used to navigate to a given wait_node"""

        def __init__(self, agent):
            """Call super to set association to wait_node"""
            super(StageDef.NavigateToTargetNode, self).__init__(agent, association='target')

    """ Communications """
    class NotifyTrigger(StageBase):
        """Used to send a message to trigger some response"""
        def __init__(self, agent, trigger, msg, colour):
            """Save initialisation details for message"""
            super(StageDef.NotifyTrigger, self).__init__(agent)
            self.trigger, self.msg, self.colour = trigger, msg, colour
        def _start(self):
            """Send trigger message"""
            super(StageDef.NotifyTrigger, self)._start()
            self.interface = self.agent.modules[self.agent['module']].interface
            self.interface.notify(self.msg)
            self.agent.cb['format_agent_marker'](self.agent, style=self.colour)
            self.interface[self.trigger] = True  # PSEUDO
        def _query(self):
            """Wait for flag to be set by message response (os #PSUEDO)"""
            success_conditions = [self.interface[self.trigger]]
            self.flag(any(success_conditions))

    """ Meta Stages """
    class Pause(StageBase):
        """Inserted on Pause for use as a 3-stage controlled blocker, used on Coordinator, Task, or Agent scopes"""
        def __repr__(self):
            """Display scopes actively contributing to the blocking"""
            return "%s(C%s|T%s|A%s)" % (self.get_class(), int(self.pause_state['Coord']), int(self.pause_state['Task']), int(self.pause_state['Agent']))
        def __init__(self, agent, format_agent_marker):
            """Initialise blocking properties"""
            super(StageDef.Pause, self).__init__(agent)
            logmsg(category="DTM", msg="      | pause init")
            self.agent.registration = False
            self.pause_state = {'Coord':False, 'Task':False, 'Agent':False}
            self.format_agent_marker = format_agent_marker
        def _start(self):
            """On start, cancel any active navigation"""
            super(StageDef.Pause, self)._start()
            if hasattr(self.agent, 'navigation_interface'): self.agent.navigation_interface.cancel_execpolicy_goal()
            #TODO: set an agent function for generic definition of pausing?
        def _query(self):
            """Continue once all blocking stages are False"""
            # success_conditions = [self.agent.registration]
            success_conditions = [not (True in self.pause_state.values())]
            self.flag(any(success_conditions))
        def _end(self):
            """On end, reenable registration"""
            self.agent.registration = True
            self.format_agent_marker(self.agent, style='')
    class Exit(StageBase):
        """Used for controlled removal of agent connections"""
        def _start(self):
            """Unregister agent and cancel any accociated tasks"""
            super(StageDef.Exit, self)._start()
            self.agent.registration = False
            for task in self.agent.task_buffer:
                self.agent.set_interrupt('reset', task['module'], task['id'], "Task")
        def _query(self):
            """Continue once all tasks are emoved from the buffer"""
            success_conditions = [len(self.agent.task_buffer) == 0]
            self.flag(any(success_conditions))
        def _end(self):
            """Set marker to black and initiate disconnection interruption-"""
            super(StageDef.Exit, self)._end()
            self.agent.cb['format_agent_marker'](self.agent, 'black')
            self.agent.set_interrupt('disconnect', 'base', self.agent['id'], "Task")
