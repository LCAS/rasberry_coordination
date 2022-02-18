"""Base"""

from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, get_param
from std_msgs.msg import Bool, String as Str
from diagnostic_msgs.msg import KeyValue
import strands_executive_msgs.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.srv import AgentNodePair
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
                    task.state = init().__repr__().replace('()','').split('.')[-1]

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
            [logmsg(category="TOC", msg="    | %s\t-- %s [%s,%s]" % (t.task_id, t.state.replace("Idle", "\033[01;32mIdle\033[0m"), t.initiator_id, t.responder_id)) for t in task_list.tasks]
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
            self.notify("INIT")

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
            state, tunnel, row, edge, task, robot = self.msg.value.split('-')
            nodes = []
            if task != module: return (None,None)
            elif tunnel == "select": return (None,None)
            elif edge != "all":
                task_scope = 'edge'
                nodes = ["tall-t%s-r%s-c%s" % (tunnel, row, e) for e in edge.split('>')]
            elif row != "all": task_scope = 'row'
            elif tunnel != "select": task_scope = 'tunnel'
            return (task_scope, {
                'tunnel': 'tall-t'+tunnel,
                'row': 'tall-t'+tunnel+'-r'+row,
                'edge': edge,
                'nodes': nodes,
                'robot': robot,
                'scope': task_scope})


    class robot(object):
        def __init__(self, agent, Type):
            self.agent = agent
            self.agent.navigation_interface = Type
            self.disconnect_sub = Subscriber('/%s/base/disconnect' % agent.agent_id, Str, self.disconnect)
        def disconnect(self, msg):
            logmsg(category="Task", id=self.agent.agent_id, msg="Request to disconnect")
            node_id = msg.data or self.agent.goal or self.agent.location(accurate=True)
            self.agent.add_task(task_name='exit_at_node', contacts={"exit_node":node_id}, index=0)


    class base_robot(robot):
        def __init__(self, agent):
            super(InterfaceDef.base_robot, self).__init__(agent, Robot(agent.agent_id))


    class base_virtual_robot(robot):
        def __init__(self, agent):
            sd = fetch_property('base', 'virtual_robot_step_delay')
            VR = VirtualRobot(agent.agent_id, agent, step_delay=sd)
            super(InterfaceDef.base_virtual_robot, self).__init__(agent, VR)


    class base_human(object):
        def __init__(self, agent):
            self.agent = agent


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
                        StageDef.AssignBaseNode(agent),
                        StageDef.NavigateToBaseNode(agent),
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
                .replace("custom_tasks.","")
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
            self.action = {}
            self.summaries = {}
        def _start(self):
            """Stage start, called when this is the active stage."""
            logmsg(category="stage", id=self.agent.agent_id, msg="Begun stage %s" % self)
            self.start_time = Time.now()
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
        def _query(self):
            """Complete the stage without any condition"""
            self.flag(True)

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
            success_conditions = [self.agent.map_handler.map]
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

    class SendInfo(StageBase):
        def __init__(self, agent):
            super(StageDef.SendInfo, self).__init__(agent)
            self.action_required = True
        def _start(self):
            super(StageDef.SendInfo, self)._start()
            self.action['action_type'] = 'send_info'
            self.action['response_location'] = ''
        def _query(self):
            """Complete once action has generated a result"""
            success_conditions = [self.action['response_location'] == 'sent']
            self.flag(any(success_conditions))


    """ Idle """
    class Idle(StageBase):
        """Used to suspend activity until the agent has a task."""
        def __repr__(self):
            """Display class with idle location """
            if self.agent: return "%s(%s)" % (self.get_class(), self.agent.location())
            return self.get_class()
        def _query(self):
            """Complete once task buffer contains another task, or if battery level is low."""
            success_conditions = [len(self.agent.task_buffer) > 0]
            self.flag(any(success_conditions))

    """ Assignment-Based Task Stages (involves coordinator) """
    class Assignment(StageBase):
        """Base task for all Assignments"""
        def _start(self):
            """Set flag to perform multi-agent action"""
            super(StageDef.Assignment, self)._start()
            self.action_required = True
        def _query(self):
            """Complete once action has generated a result"""
            success_conditions = [('response_location' in self.action and self.action['response_location'] != None)]
            self.flag(any(success_conditions))
    class AssignAgent(Assignment):
        """Handler for stages based around identifying an agent of interest."""
        def __init__(self, agent, agent_type, action_style):
            """Initialise the agent's type and the action style"""
            super(StageDef.AssignAgent, self).__init__(agent)
            self.agent_type = agent_type
            self.action_style = action_style
        def _start(self):
            """Initiate action details to identify agent"""
            super(StageDef.AssignAgent, self)._start()
            self.action['action_type'] = 'find_agent'
            self.action['action_style'] = self.action_style
            self.action['agent_type'] = self.agent_type
            self.action['response_location'] = None
        def _end(self, contact_type='responder_id'):
            """ On completion, save agent contact"""
            super(StageDef.AssignAgent, self)._end()
            resp = self.action['response_location']
            self.agent[contact_type] = resp.agent_id
            self.agent['contacts'][self.agent_type] = resp


    class AssignNode(Assignment): """Handler for stages based around itdntifying a node of interest."""; pass

    """ Node identification Stages"""
    class AssignBaseNode(AssignNode):
        """Used to identify the closest available base_node."""
        def _start(self):
            """Start action to find closest base_node"""
            super(StageDef.AssignBaseNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['descriptor'] = 'base_node'
            self.action['response_location'] = None
        def _end(self):
            """Save action response to contacts"""
            self.agent['contacts']['base_node'] = self.action['response_location']
    class AssignWaitNode(AssignNode):
        """Used to identify the closest availalbe wait_node"""
        def _start(self):
            """Start action to find closest wait_node"""
            super(StageDef.AssignWaitNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['descriptor'] = 'wait_node'
            self.action['response_location'] = None
        def _end(self):
            """Save action response to contacts"""
            self.agent['contacts']['wait_node'] = self.action['response_location']

    """ Identification of Navigation Targets """
    class FindRows(AssignNode):
        """Used to generate a list of row tasks given a tunnel id."""
        def __repr__(self):
            """Display the tunnel id to generate tasks for."""
            return "%s(%s)" % (self.get_class(), self.tunnel)
        def __init__(self, agent, tunnel, response_task):
            """Save the tunnel details"""
            super(StageDef.FindRows, self).__init__(agent)
            self.tunnel = tunnel
            self.response_task = response_task
        def _start(self):
            """Initialise action details to search for rows"""
            super(StageDef.FindRows, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'rows'
            self.action['descriptor'] = self.tunnel
            self.action['response_location'] = None
        def _end(self):
            """Begin defined task for each row in action response"""
            super(StageDef.FindRows, self)._end()
            logmsg(category="stage", msg="Task to use %s rows:" % len(self.action['response_location']))
            for row in self.action['response_location']:
                logmsg(category="stage", msg="    - extending stage list to include row: %s" % row)
                self.agent.extend_task(task_name=self.response_task, task_id=self.agent['id'], details={'row': row})
    class FindRowEnds(AssignNode):
        """Used to identify the two ends of a given row."""
        def __repr__(self):
            """Display the id for the row of interest"""
            return "%s(%s)" % (self.get_class(), self.row)
        def __init__(self, agent, row):
            """Save the row id of interest"""
            super(StageDef.FindRowEnds, self).__init__(agent)
            self.row = row
        def _start(self):
            """Start action to search for the start and end of the row of interest"""
            super(StageDef.FindRowEnds, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'row_ends'
            self.action['descriptor'] = self.row
            self.action['response_location'] = None
        def _end(self):
            """Save row ends to the contacts dictionary"""
            super(StageDef.FindRowEnds, self)._end()
            rl = self.action['response_location']
            self.agent['contacts']['row_ends'] = rl
            logmsg(category="stage", msg="Task to move from %s to %s" % (rl[0], rl[1]))
    class FindStartNode(AssignNode):
        """Used to identify of two nodes, which one is closest ot the agent."""
        def __repr__(self):
            """Display row ends in the repr"""
            if 'row_ends' in self.agent['contacts'] and self.agent['contacts']['row_ends']:
                return "%s(%s)" % (self.get_class(), self.agent['contacts']['row_ends'])
            return self.get_class()
        def _start(self):
            """Start action to find which node of the ends is the closest to start from"""
            super(StageDef.FindStartNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['list'] = self.agent['contacts']['row_ends']
            self.action['response_location'] = None
        def _end(self):
            """Save the closest and furthest node to be the row start and row end"""
            super(StageDef.FindStartNode, self)._end()
            self.agent['contacts']['start_node'] = self.action['response_location']
            self.agent['contacts']['row_ends'].remove(self.action['response_location'])
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
            self.agent.cb['trigger_replan']() #ReplanTrigger
    class NavigateToAgent(Navigation):
        """Used for navigating to a given agent"""
        def _start(self):
            """Start task by setting the target to be a defined agent's current location"""
            self.target = self.agent['contacts'][self.association].location(accurate=True)
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
