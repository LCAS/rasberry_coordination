""" Task Stages:
- All stages must inherit from StageBase.
"""

#Standard Stage Methods:
"""
Standard Stage Methods:
    - name: __init__
      description: Define basic local properties
      super: True

    - name: _start_
      description: 
      super: True

    - name: _query_
      description:

    - name: _end
      description:
      super: True

    - name: __repr__
      description: 
    - name: _get_class
      description: 
    - name: _summary
      description: 
      
    - name: _notify_start
      description: 
    - name: _notify_end
      description: 
"""

#Standard Stage Attributes:
"""
Standard Stage Attributes:
    - name: agent
      description:
    - name: action_required
      description:
    - name: route_required
      description:
    - name: stage_complete
      description:
    - name: new_stage
      description:
      
    - name: target
      description:
      
    - name: action
      description: Dictionary of details for performing actions
    - name: summary
      description: Dictionary containing descriptiosn of customised functions
      
    - name: start_time
      description: 
      

"""

#Subclass Inheritance Tree
"""
BaseStage.inheritance:
    StartTask:
    TargetedMovement:
        EdgeUVTreatment:
        RowUVTreatment:
    Await:
        AwaitAgentArrival:
        AwaitAgentDismissal:
        AwaitLoading:
        AwaitUnloading:
    Wait:
        WaitDuration:
        Pause:
"""

#
"""
All task specific details must be saved in agent.task_details
Exceptions:
    Non-Task-Based Attributes:
        - location
        - task_details #not directly accessed
    Non-Task-Based Methods:
        - .flag()
        - .new_task()   # 
        - .set_status() #communication
    Values which must persist beyond the end of task
        - store.request_admittance
        

- target   (standard for planning)
- ...
- admittance (requied for communication)

Interference with tasks should not be direct between agents.
- advencement achieved by agents with their own _query
- modify agent.tray_present, not Stage.stage_complete

Methods called within coordinator:
- closest_wait_node()
- closest_storage() #closest_target
- closest_robot()   #closest_target
"""

"""
Rules for Custom Stages:
- all cross-stage details must be stored in Stage.agent.task_details
- all communication must be limited to the Stage._notify* methods
- all overloading of __init__ must include a call to super first
- all overloading of _start must include a call to super
- all overloading of _end must include a call to super

Notes:
- __init__ is called when the task stages are added to an agent
- __repr__ can be overloaded to include more information
- _start is called once when the stask is at the head of task_stage_list
- _start can be called a second time by setting Stage.new_stage to True
- _query is called on every iteration of Coordinator.run, except when task_progression is paused
- 

(Tim Peters, The Zen of Python):
"Special cases aren't special enough to break the rules." / "Although practicality beats purity."
-> `python -c "import this"`
"""

from pprint import pprint
from copy import deepcopy
from std_msgs.msg import String as Str
import strands_executive_msgs.msg
import std_msgs.msg
from std_msgs.msg import Bool
import std_srvs.srv
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy
from rasberry_coordination.coordinator_tools import logmsg
import rasberry_coordination.srv
import rasberry_coordination.msg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption
from rasberry_coordination.task_management.__init__ import PropertiesDef as PDef

"""
python
from rospy import Publisher, init_node
init_node("temp")
from rasberry_coordination.msg import \
    TasksDetails as TasksDetailsList, \
    TaskDetails as SingleTaskDetails
ns = "/rasberry_coordination"
topic = '%s/active_tasks_details' % ns
active_tasks_pub = Publisher(topic, TasksDetailsList,
                             latch=True, queue_size=5)
task_list = TasksDetailsList()
task = SingleTaskDetails()
task.task_id = 6
task.state = "CALLED"
task.picker_id = 'picker01'
task.robot_id = 'thorvald_002'
task_list.tasks.append(task)
active_tasks_pub.publish(task_list)
# """

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

        """ Short-Definition Convenience Functions """
        def UpdateTaskList(self):
            active_task_list = self.generate_active_tasks_list();

            if self.previous_task_list != active_task_list and active_task_list.tasks:
                self.previous_task_list = active_task_list
                self.active_tasks_pub.publish(active_task_list)
        def EndTask(self, E):
            logmsg(level="error", msg="Tasks ended: %s"%E)
            active_task_list = self.generate_active_tasks_list();
            active_task_list.tasks = [t for t in active_task_list.tasks if t.task_id not in E]
            for task_id in E:
                active_task_list.tasks.append(self.generate_completed_task(task_id))
            self.active_tasks_pub.publish(active_task_list)

        """ Active Task List Modifers """
        def generate_active_tasks_list(self):
            """ Publish updated list of Active Tasks to TOC """
            task_list = TasksDetailsList()
            for agent in self.coordinator.get_agents():

                # If a task exists
                if agent['task_id']:

                    # If task is already added, move on
                    if agent['task_id'] in [T.task_id for T in task_list.tasks]: continue

                    # Get task details
                    task = SingleTaskDetails()
                    task.task_id = agent['task_id']
                    task.state = agent().__repr__().replace('()','').split('.')[-1]

                    # Assign agents to the task
                    task.initiator_id = agent.initiator_id
                    task.responder_id = agent.responder_id

                    # Add task to list
                    task_list.tasks.append(task)

            logmsg(category="TOC", msg="Active Tasks:")
            [logmsg(category="TOC", msg="----- %s | %s [%s,%s]" % (t.task_id, t.state, t.initiator_id, t.responder_id)) for t in task_list.tasks]
            return task_list
        def generate_completed_task(self, task_id=None):
            task = SingleTaskDetails()
            task.task_id = task_id
            task.state = "CANCELLED"
            return task

        """ Dynamic Task Management """
        def InterruptTask(self, m):
            #For targets, set interruption to [toc_cancel|toc_pause|toc_unpause]
            logmsg(category="DTM", id="toc", msg="Interruption made on TOC channels of type: %s" % m.interrupt)
            A = {a.agent_id:a for a in self.coordinator.get_agents()}


            interrupt_keys = {"pause":"toc_pause"
                      ,"resume":"toc_unpause"
                      ,"reset":"toc_cancel"
                      ,"cancel":"toc_cancel"
                      }
            m.interrupt = interrupt_keys[m.interrupt];

            if m.target in ["", "-"]:
                # Modify all tasks
                logmsg(category="DTM", msg="    - to affect all agents.")
                [a.set_interrupt(m.interrupt, a.task_module, a['task_id'], quiet=True) for a in A.values() if a['task_id']]

            elif m.target in A:
                # Modify specific agent's task
                logmsg(category="DTM", msg="    - to affect agent: %s." % m.target)
                A[m.target].set_interrupt(m.interrupt, A[m.target].task_module, A[m.target]['task_id'], quiet=True)

            else:
                # Modify all agents on specific task
                logmsg(category="DTM", msg="    - to affect task: %s." % m.target)
                [a.set_interrupt(m.interrupt, a.task_module, a['task_id'], quiet=True) for a in A.values() if a['task_id'] and a['task_id'] == m.target]

    class AgentInterface(object):
        def __init__(self, agent, responses, sub, pub):
            self.agent = agent
            self.responses = responses
            self.pub = Publisher(pub, Str, queue_size=5)
            self.sub = Subscriber(sub, Str, self.callback, agent.agent_id)
        def callback(self, msg, agent_id):  # Look into sub/feature
            # print("callback recieved for %s with %s" % (agent_id, msg))
            msg = eval(msg.data)
            if "states" in msg: return # car callback sends two msgs, this filters second #TODO: remove this
            if msg['user'] == agent_id:
                if msg['state'] in self.responses:
                    self.responses[msg['state']]()
        def notify(self, state):
            msg = Str('{\"user\":\"%s\", \"state\":\"%s\"}' % (self.agent.agent_id, state))
            # logmsg(category="TASK", msg="Publishing: (%s)" % msg)
            self.pub.publish(msg)

        def on_cancel(self, task_id, contact_id, force_release=False):
            logmsg(category='DTM', id=self.agent.agent_id, msg="    : Request made by %s to cancel task: %s" % (contact_id,task_id))

            # If the task is in the buffer, remove it
            try:
                if task_id in [task.task_id for task in self.agent.task_buffer]:
                    logmsg(category='DTM', msg="    - removing task from task_buffer")
                    self.agent.task_buffer = [t for t in self.agent.task_buffer if t.task_id != task_id]
                    return None
            except:
                logmsg(level="error", category='DTM', msg="    :    - task_id not attribute in item in task_buffer?")
                print(self.agent.task_buffer)
                #TODO: it seems as though restart task does not retain a task id

            # If task is active, perform appropriate cancellations for contacts
            old_id = self.agent['task_id']
            if self.agent['task_id'] == task_id:
                if force_release:
                   TaskDef.release_task(self.agent)
                else:
                    if any([contact_id.startswith(option) for option in self.release_triggers]): TaskDef.release_task(self.agent)
                    if any([contact_id.startswith(option) for option in self.restart_triggers]): TaskDef.restart_task(self.agent)
            return old_id

class TaskDef(object):
    """ Runtime Method for Custom Task Definitions """
    @classmethod
    def load_details(cls, details):
        c=deepcopy(details)
        return c
    @classmethod
    def load_task(cls, agent, task):
        agent['task_id'] = task['id']
        agent.task_name = task['name']
        agent.task_details = deepcopy(task['details'])
        agent.task_contacts = task['contacts'].copy()
        agent.task_module = task['task_module']
        agent.task_stage_list = task['stage_list']
        agent.initiator_id = task['initiator_id'] if 'initiator_id' in task else ""
        agent.responder_id = task['responder_id'] if 'responder_id' in task else ""


        logmsg(category="null")
        logmsg(category="TASK", id=agent.agent_id, msg="Active task: %s" % task['name'], speech=False)
        logmsg(category="TASK", msg="Task details:")
        for stage in task['stage_list']: logmsg(category="TASK", msg="    - %s" % stage)


    """ Runtime Method for Idle Task Definitions """
    @classmethod
    def idle(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return({'id': task_id,
                'name': "idle",
                'details': cls.load_details(details),
                'contacts': contacts.copy(),
                'task_module': 'base',
                'initiator_id': agent.agent_id,
                'responder_id': "",
                'stage_list': [
                    StageDef.StartTask(agent, task_id),
                    StageDef.Idle(agent)
                ]})
    @classmethod
    def wait_at_base(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return({'id': task_id,
                'name': "wait_at_base",
                'details': cls.load_details(details),
                'contacts': contacts.copy(),
                'task_module': 'base', # TODO: setup common task to replace this
                'initiator_id': agent.agent_id,
                'responder_id': "",
                'stage_list': [
                    StageDef.StartTask(agent, task_id),
                    StageDef.AssignBaseNode(agent),
                    StageDef.NavigateToBaseNode(agent),
                    StageDef.Idle(agent)
                ]})
    @classmethod
    def exit_at_node(cls, agent, task_id=None, details={}, contacts={}, initiator_id=""):
        return({'id': task_id,
                'name': "exit_at_node",
                'details': cls.load_details(details),
                'contacts': contacts.copy(),
                'task_module': 'base',
                'initiator_id': agent.agent_id,
                'responder_id': "",
                'stage_list': [
                    StageDef.NavigateToExitNode(agent),
                    StageDef.Exit(agent)
                ]})

    """ Dynamic Task Management """
    @classmethod
    def release_task(cls, agent):
        logmsg(category="DTM", msg="    :    - releasing task %s" % (agent.task_name))
        task_name = agent.task_name
        agent.task_name = None
        agent.action = dict()
        agent.task_details = {}
        agent.task_contacts = {}
        task_module = None
        agent.task_stage_list = []
        return task_name
    @classmethod
    def restart_task(cls, agent):
        task_name = TaskDef.release_task(agent)
        logmsg(category="DTM", msg="    :    - restarting task %s" % (task_name))
        agent.add_task(task_name=task_name, index=0, quiet=True)

class StageDef(object):
    class StageBase(object):
        def __repr__(self):
            return self.get_class()
        def get_class(self):
            return str(self.__class__).replace("<class 'rasberry_coordination.task_management.","").replace("'>","")
        def __init__(self, agent):
            self.agent = agent
            self.action_required = False
            self.route_required = False
            self.route_found = False
            self.stage_complete = False
            self.new_stage = True
            self.target = None
            self.action = {}
            self.summary = {}

            self._summary()
        def _summary(self):
            placeholder = '-'
            self.summary['_start'] = placeholder
            self.summary['_notify_start'] = placeholder
            self.summary['_query'] = placeholder
            self.summary['_action'] = placeholder
            self.summary['_notify_end'] = placeholder
            self.summary['_del'] = placeholder
        def _start(self):
            """ Called whenever a new stage is set.
            Also called whenever Stage.new_stage is set to True
            """
            logmsg(category="stage", id=self.agent.agent_id, msg="Begun stage %s" % self)
            self.start_time = Time.now()
        def _notify_start(self):
            pass
        def _query(self):
            success_conditions = []  # What should be queried to tell if the stage is completed?
            self._flag(any(success_conditions))
        def _flag(self, flag):
            # TODO: this is not very good here, the state of an agent being interrupted should be handled abstractly
            if not self.agent.interruption:
                self.stage_complete = flag
        def _notify_end(self):
            pass
        def _end(self):
            self.stage_complete = False
            pass

    """ Standard Task Stages """
    class StartTask(StageBase):
        """ Called as the first stage of every active task (excluding idle tasks),
        this stage is responsible for the creation of the task_id on initialisation.

        Created following the convention of {agent_id}_{total_tasks++}. This is a
        unique key on condition that agents each have UUIDs for agent_id. No lock is
        required for this to fuction.

        Once this becomes the active task in the task_stage_list, the task_id is
        adopted by the agent as the active task_id.

        This stage completes without any conditions.
        """
        def __init__(self, agent, task_id=None):
            super(StageDef.StartTask, self).__init__(agent)
            self.task_id = task_id if task_id else "%s_%s" % (self.agent.agent_id.replace('thorvald','T').replace('picker','P').replace('storage','S'), self.agent.total_tasks) #TODO: this replace nest needs removing
            self.agent.total_tasks += 1
        def _start(self):
            self.agent.task_details = {}
            super(StageDef.StartTask, self)._start()
            self.agent['task_id'] = self.task_id #Set task_id as active_task_id for agent
            self.agent['start_time'] = Time.now()
        def _query(self):
            self._flag(True)
        def _summary(self):
            super(StageDef.StartTask, self)._summary()
            self.summary['_start'] = "adopt active task_id"
            self.summary['_query'] = "return true"
    class WaitForLocalisation(StageBase):
        def _query(self):
            success_conditions = [self.agent.location() is not None]
            self._flag(any(success_conditions))
        def _end(self):
            super(StageDef.WaitForLocalisation, self)._end()
            logmsg(category="stage", msg="Localisation achieved %s" % self.agent.location())

    """ Idle """
    class Idle(StageBase):
        def _query(self):
            success_conditions = [len(self.agent.task_buffer) > 0,
                                  self.agent.interfaces['health_monitoring'].battery_low() if 'health_monitoring' in self.agent.interfaces else False] #this needs to be removed
            self._flag(any(success_conditions))
        def _summary(self):
            # logmsg(level='error', id=self.agent.agent_id, msg=self.agent.properties, speech=False)
            super(StageDef.Idle, self)._summary()
            self.summary['_query'] = 'len(task_buffer) > 0'

    """ Assignment-Based Task Stages (involves coordinator) """
    class Assignment(StageBase): #Define as ABC
        def _start(self):
            super(StageDef.Assignment, self)._start()
            self.action_required = True
        def _query(self):
            success_conditions = [('response_location' in self.action and self.action['response_location'] != None)]
            self._flag(any(success_conditions))
        def _summary(self):
            super(StageDef.Assignment, self)._summary()
            self.summary['_start'] = "load service requirements"
    class AssignAgent(Assignment): pass
    class AssignNode(Assignment): pass
    class AssignBaseNode(AssignNode):
        def _start(self):
            super(StageDef.AssignBaseNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['descriptor'] = 'base_node'
            self.action['response_location'] = None
        def _end(self):
            self.agent.task_contacts['base_node'] = self.action['response_location']

    """ Navigation Controllers for Courier """
    class Navigation(StageBase):
        def __repr__(self):
            if self.target:
                return "%s(%s)"%(self.get_class(), self.target.replace('WayPoint','WP'))
            else:
                return "%s" % (self.get_class())
        def __init__(self, agent, association):
            super(StageDef.Navigation, self).__init__(agent)
            self.association = association
            self.target = None
        def _start(self):
            super(StageDef.Navigation, self)._start()
            self.route_found = False
            self.route_required = True
            logmsg(category="stage", id=self.agent.agent_id, msg="Navigation from %s to %s is begun." % (self.agent.location(accurate=True), self.target))
        def _query(self):
            success_conditions = [self.agent.location(accurate=True) == self.target]
            self._flag(any(success_conditions))
        def _end(self):
            logmsg(category="stage", id=self.agent.agent_id, msg="Navigation from %s to %s is completed." % (self.agent.location(accurate=True), self.target))
            # self.agent.temp_interface.cancel_execpolicy_goal()
            self.agent.cb['trigger_replan']() #ReplanTrigger
    class NavigateToAgent(Navigation):
        def _start(self):
            super(StageDef.NavigateToAgent, self)._start()
            self.target = self.agent.task_contacts[self.association].location(accurate=True)
    class NavigateToNode(Navigation):
        def _start(self):
            super(StageDef.NavigateToNode, self)._start()
            self.target = self.agent.task_contacts[self.association]

    """ Navigation SubSubclasses """
    class NavigateToBaseNode(NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToBaseNode, self).__init__(agent, association='base_node')
        def _query(self):
            success_conditions = [self.agent.location(accurate=True) == self.target]
            self._flag(any(success_conditions))
    class NavigateToExitNode(NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToExitNode, self).__init__(agent, association='exit_node')
        def _query(self):
            success_conditions = [self.agent.location(accurate=True) == self.target]
            self._flag(any(success_conditions))

    """ Routing Recovery Behviour """
    class AssignWaitNode(AssignNode):
        def _start(self):
            super(StageDef.AssignWaitNode, self)._start()
            self.action['action_type'] = 'find_node'
            self.action['action_style'] = 'closest'
            self.action['descriptor'] = 'wait_node'
            self.action['response_location'] = None
        def _end(self):
            self.agent.task_contacts['wait_node'] = self.action['response_location']
    class NavigateToWaitNode(NavigateToNode):
        def __init__(self, agent): super(StageDef.NavigateToWaitNode, self).__init__(agent, association='wait_node')

    """ Meta Stages """
    class Pause(StageBase):
        def _start(self):
            self.agent.registration=False
            if hasattr(self.agent, 'temp_interface'):
                self.agent.temp_interface.cancel_execpolicy_goal()
            pass
        def _query(self):
            success_conditions = [self.agent.registration]
            self._flag(any(success_conditions))
    class Unregister(StageBase): pass
    class Exit(StageBase):
        def _start(self):
            super(StageDef.Exit, self)._start()
            self.agent.registration = False
            for task in self.agent.task_buffer:
                self.agent.set_interrupt('force_cancel_task', task['task_module'], task['id'])
        def _query(self):
            success_conditions = [len(self.agent.task_buffer) == 0];
            self._flag(any(success_conditions))
        def _end(self):
            super(StageDef.Exit, self)._end()
            self.agent.cb['format_agent_marker'](self.agent.agent_id, 'black')
            self.agent.set_interrupt('delete_agent', '', '')
