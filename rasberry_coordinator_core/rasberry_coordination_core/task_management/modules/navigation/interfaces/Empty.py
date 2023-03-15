# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination_core.task_management.modules.base.interfaces.Interface import iFACE as Interface
from rasberry_coordination_core.task_management.__init__ import Stages
from rasberry_coordination_core.task_management.containers.Task import TaskObj as Task
from rasberry_coordination_core.logmsg_utils import logmsg

from rasberry_coordination_core.topomap_management.occupancy import OccupancyFilters
from diagnostic_msgs.msg import KeyValue

# Automanaged by rasberry_coordination_core.task_management.__init__.load_modules
# Interface class must be named `iFACE` to be recognised for import
# It will then be identifiable by its Interfaces[<<module>>][<<filename>>]
class iFACE(Interface):
    def occupation(self):
        """ Filter map based on occupation types associated to node name """
        if not self.agent.location.has_presence:
            return []

        #Get location name
        node = self.agent.location(accurate=False)
        type_list = ["self"]

        # Apply filters
        logmsg(category="occupy", id=self.agent.agent_id, msg="Occupation:")
        nodes_to_filter = []
        for typ in type_list:
            method = getattr(OccupancyFilters, typ)
            nodes_to_filter += method(self.agent.map_handler.global_map, self.agent.map_handler.global_node_list, node)
            logmsg(category="occupy", msg="  | %s: %s"%(typ, str(nodes_to_filter)))

        return nodes_to_filter

