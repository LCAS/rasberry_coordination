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
        self.info = info #['send_info', 'find_row_ends', 'find_rows']
        self.list = list

        self.response = None
        self.silence = False

class ActionManager(object):
    def __init__(self, agent_manager, route_finder, special_nodes, get_agents_fcn):
        self.agent_manager = agent_manager
        self.route_finder = route_finder
        self.special_nodes = special_nodes
        self.get_agents_fcn = get_agents_fcn
        pass

    """ Services offerd by Coordinator to assist with tasks """
    def offer_service(self, agent):
        action = agent().action
        TP = action.type

        self.AllAgentsList = self.get_agents_fcn()  # TODO: add enter and exit commands for agent manager.agent_details.copy()?
        if TP == 'search':
            #print("action: %s" % str(action))
            list = self.get_list(agent)
            #print("list: %s" % str(list))
            item = self.get_item(agent, list)
            #print("item: %s" % str(item))
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
            return self.route_finder.planner.get_row_ends(agent, row_id)

        elif FO == 'find_rows':
            tunnel_id = action.descriptor
            return self.route_finder.planner.get_rows(agent, tunnel_id)

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
                 (a is not agent) and a.registration and (descriptor in a.roles())}

        elif GR == 'new_list_generators_go_here':
            L = {}

        return L

    def get_item(self, agent, list):
        action = agent().action
        location = agent.location()
        ST = action.style
        #print('item')
        if ST == 'closest_agent':
            new_list = {k: self.dist(v, v.location(), location) for k, v in list.items()}
            I = self.AllAgentsList[self.get_dist(new_list)]

        elif ST == 'closest_node':
            #print('closest_node')
            new_list = {n: self.dist(agent, n, agent.location()) for n in list}
            I = self.get_dist(new_list)

        elif ST == 'new_identifications_go_here':
            I = None

        return I

    """ Action Tools """

    def get_occupied_nodes(self, agent):
        print('occupied')
        AExcl = [a for _id, a in self.AllAgentsList.items() if (_id is not agent.agent_id)]
        print(AExcl)
        occupied = [a.location.current_node for a in AExcl if a.location.current_node]  # Check if node is occupied
        print(occupied)
        occupied += [a().action.response for a in AExcl if a().action and a().action.response and a.map_handler.is_node(a().action.response)]  # Include navigation targets
        print(occupied)
        occupied += [a.goal() for a in AExcl if a.goal()] # Include navigation targets
        print(occupied)
        return occupied

    def get_dist(self, dist_list):
        if dist_list:
            return min(dist_list, key=dist_list.get)
        return None

    def dist(self, agent, start_node, goal_node):
        return agent.map_handler.get_route_length(agent, start_node, goal_node)



"""
(type, grouping, descriptor, style)
(type, info)

Info
    sendinfo (info, send_info)
    findrows (info, find_rows)
    findrowends (info, find_row_ends)
Search
    Node
        descriptor-
            assignbasenode (search, node_descriptor, base_node, closest_node)
            assignwaitnode (search, node_descriptor, wait_node, closest_node)
            assignchargenode (search, node_descriptor, charge_node, closest_node)
        list-
            findstartnode (search, node_list, ~, closest_node)
    Agent
        descriptor-
            assignfieldcourier (search, agent_descriptor, courier, closest_agent)
            assignfieldstorage (search, agent_descriptor, storage, closest_agent)
            assignscanner (search, agent_descriptor, scanner, closest_agent)
        list-
            acceptfieldcourier (search, agent_list, ~, closest_agent)
"""

