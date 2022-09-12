"""Base"""

from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, get_param
from std_msgs.msg import Bool, String as Str
from diagnostic_msgs.msg import KeyValue
import strands_executive_msgs.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.action_management.manager import ActionDetails
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
            for agent in self.coordinator.agent_manager.get_agent_list_copy().values():

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
            logmsg(category="null")
            logmsg(category="DTM", id="toc", msg="Interruption made on TOC channels of type: %s" % m.interrupt)
            A = {a.agent_id:a for a in self.coordinator.get_agents()}

            if m.scope in [0, "Coord", "Coordinator"]:
                # Modify all tasks
                logmsg(category="DTM", msg="    - to affect all agents.")
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

