import traceback
from std_msgs.msg import String as Str
from rospy import Publisher
from rasberry_coordination.coordinator_tools import logmsg

class InteractionDetails(object):
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

class InteractionManager(object):
    def __init__(self, agent_manager, routing_manager, special_nodes):
        self.agent_manager = agent_manager
        self.routing_manager = routing_manager
        self.special_nodes = special_nodes
        pass

    """ Services offerd by Coordinator to assist with tasks """
    def offer_service(self, agent):
        interaction = agent().interaction
        TP = interaction.type

        self.AllAgentsList = self.agent_manager.agent_details.copy()  # TODO: try enter and exit instead of .copy()?
        if TP == 'search':
            try:
                list = self.get_list(agent)
                item = self.get_item(agent, list)
            except Exception as e:
                print("Error on interaction manager:")
                print(traceback.format_exc())
                item = None
            interaction.response = item
        elif TP == 'info':
            resp = self.get_info(agent)
            interaction.response = resp

        if interaction.response:
            logmsg(category="action", id='INTERACTION', msg="Performing Interaction Search")
            [logmsg(category="action", msg="    - %s: %s"%(k,v)) for k,v in interaction.__dict__.items() if v]
            if TP=="search":
                logmsg(category="action", msg="    - list: %s" % str(list))
            logmsg(category="action", msg="    - RESULT: %s" % interaction.response)
        elif not interaction.silence:
            logmsg(category="action", id='INTERACTION', msg="Performing Interaction Search")
            [logmsg(category="action", msg="    - %s: %s"%(k,v)) for k,v in interaction.__dict__.items() if v]
            logmsg(category="action", msg="    - Interaction result not found, will notify when result found")
            interaction.silence = True

        self.AllAgentsList = None

    def get_list(self, agent):
        # Get list from given discription
        interaction = agent().interaction
        GR = interaction.grouping

        if GR == 'node_list':
            # Use given list of nodes
            L = interaction.list

        elif GR == 'agent_list':
            # Use given list of agents
            L = {agent_id: self.AllAgentsList[agent_id] for agent_id in interaction.list if agent_id in self.AllAgentsList}

        elif GR == 'node_descriptor':
            # Generate list of nodes matching descriptor (special nodes listed in coordinator config)
            descriptor = interaction.descriptor
            occupied = self.get_occupied_nodes(agent)
            print("occupied: %s" % str(occupied))
            L = [n['id'] for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in occupied)]
            print("L %s"%str(L))

        elif GR == 'agent_descriptor':
            # Generate list of agents based on some characteristics
            descriptor = interaction.descriptor
            r = descriptor['interface']
            descriptor['interface'] = r if type(r) == type([]) else [r]
            L = {a.agent_id: a for a in self.AllAgentsList.values() if
                     (a is not agent) and #not the agent making the call
                     (a.registration) and #agent is registered
                     (a().accepting_new_tasks) and #agent is accepting new tasks / active task is interruptable
                     (descriptor['module'] in a.modules) and #agent has the required module
                     (str(a.modules[descriptor['module']].interface) in descriptor['interface']) #agent is of the type required
                 #TODO: we need to make sure here that the robot has not been assigned to a picker on the same cycle
                 #and a.id not in self.cycle_repsonse? #todo: this will have tons of problems...
                 }
            #TODO make accepitng tasks a different generator

        elif GR == 'head_nodes':
            # Generate list of nodes based on format of name
            L = [float(n.split("-c")[0][1:]) for n in agent.map_handler.empty_node_list if n.endswith('ca')]

        elif GR == 'new_list_generators_go_here':
            L = {}

        return L


    def get_item(self, agent, list):
        # Get item from given list
        interaction = agent().interaction
        location = agent.location()
        ST = interaction.style
        if ST == 'named_agent':
            i = list.keys()[0]
            I = self.AllAgentsList[i] if i in self.AllAgentsList else None

        elif ST == 'closest_agent':
            # Find closet agent in list
            new_list = {k: self.dist(v, v.location(), location) for k, v in list.items()}
            i = self.get_dist(new_list)
            I = self.AllAgentsList[i] if i in self.AllAgentsList else None

        elif ST == 'closest_node':
            # Find closet node in list
            new_list = {n: self.dist(agent, n, agent.location()) for n in list}
            I = self.get_dist(new_list)
            print(I)

        elif ST == 'head_node_allocator':
            PLoc = {a.agent_id:float(a.location().split("-c")[0][1:]) for a in self.AllAgentsList.values() if a.registration and a.location() and ('-c' in a.location()) and (str(a.modules['transportation'].interface) == 'picker')}
            if PLoc:
                from ideal_parking_spot import ideal_parking_spot as ips
                parking_spots = ["r%s-ca"% spot for spot in ips(list, PLoc)]
                occupied = self.get_occupied_nodes(agent)
                new_list = [spot for spot in parking_spots if spot not in occupied]
                I = new_list[0]
            else:
                I = None

        elif ST == 'new_identifications_go_here':
            I = None

        else:
            I = None

        return I


    def get_info(self, agent):
        interaction = agent().interaction
        FO = interaction.info

        if FO == 'send_info':
            pub1 = Publisher('/car_client/info/map', Str, queue_size=1, latch=True)
            pub1.publish(agent.map_handler.simplify())  # issues with res_map?, (but called by humans & they use full)
            pub2 = Publisher('/car_client/info/robots', Str, queue_size=1, latch=True)
            pub2.publish(self.agent_manager.simplify())
            return 'sent'

        elif FO == 'find_row_ends':
            row_id = interaction.descriptor
            return self.routing_manager.planner.get_row_ends(agent, row_id)


    """ Interaction Tools """

    def get_occupied_nodes(self, agent):
        logmsg(level='error', category="occupy", id="INTERACTION", msg="Occupied Nodes System needs updating")
        AExcl = [a for _id, a in self.AllAgentsList.items() if (_id is not agent.agent_id)]
        occupied = [a.location.current_node for a in AExcl if a.location.current_node]  # Check if node is occupied
        occupied += [a().interaction.response for a in AExcl if a().interaction and a().interaction.response and a.map_handler.is_node(a().interaction.response)]  # Include navigation targets
        occupied += [a.goal() for a in AExcl if a.goal()] # Include navigation targets
        return occupied

    def get_dist(self, dist_list):
        if dist_list:
            return min(dist_list, key=dist_list.get)
        return None

    def dist(self, agent, start_node, goal_node):
        try:
            return agent.map_handler.get_route_length(agent, start_node, goal_node)
        except:
            print("Try-Except in manager.py for Action_Management modules")
            print(traceback.format_exc())
            return 0.0
