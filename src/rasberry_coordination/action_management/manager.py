from std_msgs.msg import String as Str
from rospy import Publisher
from rasberry_coordination.coordinator_tools import logmsg

class ActionDetails(object):
    """TODO: move to encapsulators"""
    def __init__(self, type, grouping=None, descriptor=None, style=None, info=None, list=None):
        self.type = type #[search, info]
        self.grouping = grouping #[node_list, agent_list, node_descriptor, agent_descriptor]
        self.descriptor = descriptor #'base_station'
        self.style = style #closest_node, closest_agent
        self.info = info #['send_info', 'find_row_ends']
        self.list = list

        self.response = None
        self.silence = False

class ActionManager(object):
    def __init__(self, agent_manager, routing_manager, special_nodes):
        self.agent_manager = agent_manager
        self.routing_manager = routing_manager
        self.special_nodes = special_nodes
        pass

    """ Services offerd by Coordinator to assist with tasks """
    def offer_service(self, agent):
        action = agent().action
        TP = action.type

        self.AllAgentsList = self.agent_manager.agent_details.copy()  # TODO: try enter and exit instead of .copy()?
        if TP == 'search':
            list = self.get_list(agent)
            item = self.get_item(agent, list)
            action.response = item
        elif TP == 'info':
            resp = self.get_info(agent)
            action.response = resp

        if action.response:
            logmsg(category="action", msg="    - Performing action: %s" % {k: v for k, v in action.__dict__.items() if v})
            if TP=="search":
                logmsg(category="action", msg="        - list: %s" % str(list))
            logmsg(category="action", msg="    - Action result found: %s" % action.response)
        elif not action.silence:
            logmsg(category="action", msg="    - Performing action: %s" % {k: v for k, v in action.__dict__.items() if v})
            logmsg(category="action", msg="    - Action result not found, will notify when result found")
            action.silence = True

        self.AllAgentsList = None

    def get_info(self, agent):
        action = agent().action
        FO = action.info

        if FO == 'send_info':
            pub1 = Publisher('/car_client/info/map', Str, queue_size=1, latch=True)
            pub1.publish(agent.map_handler.simplify())  # issues with res_map?, (but called by humans & they use full)
            pub2 = Publisher('/car_client/info/robots', Str, queue_size=1, latch=True)
            pub2.publish(self.agent_manager.simplify())
            return 'sent'

        elif FO == 'find_row_ends':
            row_id = action.descriptor
            return self.routing_manager.planner.get_row_ends(agent, row_id)

    def get_list(self, agent):
        action = agent().action
        GR = action.grouping

        if GR == 'node_list':
            L = action.list

        elif GR == 'agent_list':
            L = {agent_id: self.AllAgentsList[agent_id] for agent_id in action.list}

        elif GR == 'node_descriptor':
            descriptor = agent().action.descriptor
            occupied = self.get_occupied_nodes(agent)
            print("occupied: %s" % str(occupied))
            L = [n['id'] for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in occupied)]

        elif GR == 'agent_descriptor':
            descriptor = agent().action.descriptor
            L = {a.agent_id: a for a in self.AllAgentsList.values() if
                 (a is not agent) and a.registration and a().accepting_new_tasks and (descriptor in a.roles())}
            #TODO make accepitng tasks a different generator

        elif GR == 'head_nodes':
            L = [n for n in agent.map_manager.empty_node_list if n.endswith('ca')]

        elif GR == 'new_list_generators_go_here':
            L = {}

        return L

    def get_item(self, agent, list):
        action = agent().action
        location = agent.location()
        ST = action.style
        if ST == 'closest_agent':
            new_list = {k: self.dist(v, v.location(), location) for k, v in list.items()}
            i = self.get_dist(new_list)
            I = self.AllAgentsList[i] if i in self.AllAgentsList else None

        elif ST == 'closest_node':
            new_list = {n: self.dist(agent, n, agent.location()) for n in list}
            I = self.get_dist(new_list)

        elif ST == 'head_node_allocator':
            PLoc = {a.agent_id: a.location.closest_node.split("-c")[0][1:] for a in self.AllAgents}
            from ideal_parking_spot import ideal_parking_spot as ips
            parking_spots = ["r%s-ca"% spot for spot in ips(PLoc, list)]
            occupied = self.get_occupied_nodes(agent)
            new_list = [spot for spot in parking_spots if spot not in occupied]
            I = new_list[0]

        elif ST == 'new_identifications_go_here':
            I = None

        return I

    """ Action Tools """

    def get_occupied_nodes(self, agent):
        AExcl = [a for _id, a in self.AllAgentsList.items() if (_id is not agent.agent_id)]
        occupied = [a.location.current_node for a in AExcl if a.location.current_node]  # Check if node is occupied
        occupied += [a().action.response for a in AExcl if a().action and a().action.response and a.map_handler.is_node(a().action.response)]  # Include navigation targets
        occupied += [a.goal() for a in AExcl if a.goal()] # Include navigation targets
        return occupied

    def get_dist(self, dist_list):
        if dist_list:
            return min(dist_list, key=dist_list.get)
        return None

    def dist(self, agent, start_node, goal_node):
        return agent.map_handler.get_route_length(agent, start_node, goal_node)

