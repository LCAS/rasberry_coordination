from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy

from std_msgs.msg import Bool, String as Str, Empty as Emp
import strands_executive_msgs.msg

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption

import yaml
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch
from topological_navigation.tmap_utils import get_node_from_tmap2 as GetNode, get_distance_to_node_tmap2 as GetNodeDist

class ModuleObj(object):

    def __repr__(self):
        return "Module( name:%s | role:%s | interface:%s )" % (self.name, self.role, self.interface!=None)

    def __init__(self, agent, name, role, details):
        #logmsg(category="module", msg="%s (%s)"%(name.upper(),role.upper()))
        self.agent = agent
        self.name = name
        self.role = role
        from rasberry_coordination.task_management.__init__ import Interfaces, PropertiesDef
        self.interface = Interfaces[name][role](agent=agent, details=details)
        self.properties = PropertiesDef[name] if name in PropertiesDef else dict()
        self.details = details

    def add_init_task(self):
        logmsg(category="module", msg="%s" % self.name.upper())
        logmsg(category="module", msg="    | Role: %s" % self.role)
        logmsg(category="module", msg="    | Searching for init task:")
        self.agent.add_task(module=self.name, name='init')

