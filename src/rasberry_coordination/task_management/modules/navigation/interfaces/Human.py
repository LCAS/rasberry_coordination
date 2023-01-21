from rasberry_coordination.task_management.modules.base.interfaces.Interface import Interface
from rasberry_coordination.topomap_management.occupancy import OccupancyFilters
from rasberry_coordination.coordinator_tools import logmsg


class Human(Interface):

    def occupation(self):
        """ Filter map based on occupation types associated to node name """
        if not self.agent.location.has_presence:
            return []

        #Get location name
        node = self.agent.location(accurate=False)
        type_list = ["self"]

        # Apply filters
        logmsg(category="occupy", id=self.agent.agent_id, msg="Occupation: %s + %s"%(node, type_list))
        nodes_to_filter = []
        for typ in type_list:
            method = getattr(OccupancyFilters, typ)
            nodes_to_filter += method(self.agent.map_handler.global_map, self.agent.map_handler.global_node_list, node)
            logmsg(category="occupy", msg="  - %s: %s"%(typ, str(nodes_to_filter)))

        return nodes_to_filter

