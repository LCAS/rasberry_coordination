from copy import deepcopy
from rospy import Time, Duration, Subscriber, Service, Publisher, Time, ServiceProxy

from std_msgs.msg import Bool, String as Str, Empty as Emp
import strands_executive_msgs.msg

from rasberry_coordination.coordinator_tools import logmsg
from rasberry_coordination.msg import TasksDetails as TasksDetailsList, TaskDetails as SingleTaskDetails, Interruption

import yaml
from topological_navigation.route_search2 import TopologicalRouteSearch2 as TopologicalRouteSearch
from topological_navigation.tmap_utils import get_node_from_tmap2 as GetNode, get_distance_to_node_tmap2 as GetNodeDist


class TaskObj(object):

    def __repr__(self):
        return "Task( id:%s | module:%s | name:%s | init:%s | resp:%s | #stages:%s )" % \
               (self.id, self.module, self.name, self.initiator_id, self.responder_id, len(self.stage_list))

    def __init__(self, id=None, name=None, module=None, details=None, contacts=None, initiator_id=None, responder_id=None, stage_list=None):
        self.id = str("...") if not id else id
        self.name = str() if not name else name
        self.module = str() if not name else module
        self.details = dict() if not details else details
        self.contacts = dict() if not contacts else contacts
        self.initiator_id = str() if not initiator_id else initiator_id
        self.responder_id = str() if not responder_id else responder_id
        self.stage_list = list() if not stage_list else stage_list

    def __getitem__(self, key): return self.__getattribute__(key) if key in self.__dict__ else None
    def __setitem__(self, key, val): self.__setattr__(key,val)


