from rospy import Publisher, Subscriber
from rasberry_coordination.msg import Task as TaskMsg, AllSchedulableTasks, SchedulableTasks
from rasberry_coordination.task_management.containers.Task import TaskObj as Task

from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.task_management.__init__ import Stages
from rasberry_coordination.coordinator_tools import logmsg


class SchedulerManager(Interface):

    def __init__(self, agent, details=None):
        super(SchedulerManager, self).__init__(agent=agent, details=details)
        self.schedule_sub = Subscriber('~scheduler/start_task', TaskMsg, self.schedule_new_task)
        self.available_schedulers_pub = Publisher('~scheduler/available', AllSchedulableTasks, latch=True, queue_size=1)

    def list_schedulable_tasks(self):
        ast = AllSchedulableTasks()
        for name, module in self.agent.modules.items():
            if not name.startswith('rasberry_') or not name.endswith('_pkg'): continue
            sch = SchedulableTasks()
            sch.action = '_'.join(name.split('_')[1:-1])
            sch.criterian = module.interface.list_schedulable_tasks()
            ast.list.append(sch)
        self.available_schedulers_pub.publish(ast)

    def schedule_new_task(self, msg):
        pkg = "rasberry_%s_pkg"%msg.credentials.action
        interface_function = "assign_"+msg.criteria.task_name

        details = {'criteria':msg.criteria, 'schedule':msg.schedule}
        act = ''.join([b[0] for b in msg.credentials.action.split('_')]).upper()
        tid = "Sch-%s_%s" % (act, msg.credentials.id)

        logmsg(category="SCHEDU", id=self.agent.agent_id, msg="Scheduling a new task")
        self.agent.add_task(module=pkg, name=interface_function, task_id=tid, details={'msg': msg})
        #PSUEDO: Stages['assignment']['AssignTask'](self.agent, details)
