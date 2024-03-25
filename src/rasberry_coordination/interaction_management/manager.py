import traceback
from std_msgs.msg import String as Str
from rospy import Publisher
from rasberry_coordination.coordinator_tools import logmsg

class InteractionDetails(object):
    """TODO: move to encapsulators"""
    def __init__(self, type, grouping=None, descriptor=None, style=None, style_details=None, info=None, list=None):
        self.type = type #[search, info]

        ### search
        # grouping
        self.grouping = grouping #[node_list, agent_list, node_descriptor, agent_descriptor]
        self.descriptor = descriptor #'base_station'
        self.list = list
        # selection
        self.style = style #closest_node, closest_agent
        self.style_details = style_details
        # result
        self.response = None

        ### info
        self.info = info #['send_info', 'find_row_ends']

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

        self.AllAgentsList = self.agent_manager.agent_details.copy()
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
            logmsg(category="action", id='MEDIATOR', msg="Performing Interaction Search for %s"%agent.agent_id)
            [logmsg(category="action",    msg="   | %s: %s"%(k,v)) for k,v in interaction.__dict__.items() if v]
            if TP=="search":
                logmsg(category="action", msg="   | list: %s" % str(list))
            logmsg(category="action",     msg="   | RESULT: %s" % interaction.response)
        elif not interaction.silence:
            logmsg(category="action", id='MEDIATOR', msg="Performing Interaction Search %s"%agent.agent_id)
            [logmsg(category="action",    msg="   | %s: %s"%(k,v)) for k,v in interaction.__dict__.items() if v]
            logmsg(category="action",     msg="   | Interaction result not found, will notify when result found")
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
            L = {a.agent_id: a for a in self.AllAgentsList.values() if
                    (a.agent_id in interaction.list) and
                    (a.registration) and
                    (a().accepting_new_tasks)
                }
            if not L: return "empty"

        elif GR == 'forced_agent_list':
            # Use given list of agents
            L = {a.agent_id: a for a in self.AllAgentsList.values() if
                    (a.agent_id in interaction.list)
                }
            if not L: return "empty"

        elif GR == 'node_descriptor':
            # Generate list of nodes matching descriptor (special nodes listed in coordinator config)
            descriptor = interaction.descriptor
            occupied = self.get_occupied_nodes(agent)
            L = [n['id'] for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in occupied)]

        elif GR == 'agent_descriptor':
            # Generate list of agents based on some characteristics
            descriptor = interaction.descriptor

            # Format some shorthand
            m = descriptor['module']
            r = descriptor['interface']
            descriptor['interface'] = r if type(r) == type([]) else [r]
            r = descriptor['interface']

            # Generate list of all agents which could be of interest
            L = {a.agent_id: a for a in self.AllAgentsList.values() if (a is not agent)} #not making the call

            # Filter list by filtering out ones of invalid forms
            L = {k:v for k,v in L.items() if m in v.modules and (str(v.modules[m].interface) in r)}

            # Return here, if no viable agents in the system, rather than none currently avaiable
            if not L: return "empty"

            # Filter list by filtering out ones not accepting new tasks
            #print('\n\nChecking if agent is accepting new tasks...')
            #print([[k,v().accepting_new_tasks] for k,v in L.items()])
            L = {k:v for k,v in L.items() if (v.registration) and (v().accepting_new_tasks)}
            #print([[k,v().accepting_new_tasks] for k,v in L.items()])

        elif GR == 'forced_agent_descriptor':
            # Generate list of agents based on some characteristics
            descriptor = interaction.descriptor

            # Format some shorthand
            m = descriptor['module']
            r = descriptor['interface']
            descriptor['interface'] = r if type(r) == type([]) else [r]
            r = descriptor['interface']

            # Generate list of all agents which could be of interest
            L = {a.agent_id: a for a in self.AllAgentsList.values() if (a is not agent)} #not making the call

            # Filter list by filtering out ones of invalid forms
            L = {k:v for k,v in L.items() if m in v.modules and (str(v.modules[m].interface) in r)}

            # Return here, if no viable agents in the system, rather than none currently avaiable
            if not L: return "empty"

            # Filter list by filtering out ones not accepting new tasks
            L = {k:v for k,v in L.items()}

        elif GR == 'head_nodes':
            # Generate list of nodes based on format of name
            L = [float(n.split("-c")[0][1:]) for n in agent.map_handler.empty_node_list if n.endswith('ca')]

        elif GR == 'new_list_generators_go_here':
            L = dict()

        return L


    def get_item(self, agent, list):
        # Get item from given list
        interaction = agent().interaction
        location = agent.location()
        ST = interaction.style
        SD = interaction.style_details

        if list == "empty":
            return "empty"

        elif ST == 'named_agent':
            i = list.keys()[0] if list else ''
            I = self.AllAgentsList[i] if i in self.AllAgentsList else None

        elif ST == 'closest_agent':
            # Find closet agent in list
            if SD and 'nodes' in SD and SD['nodes']:
                location = SD['nodes']
            new_list = {k: self.dist(v, v.location(), location) for k, v in list.items()}
            i = self.get_dist(new_list)
            I = self.AllAgentsList[i] if i in self.AllAgentsList else None

            #TODO: this will still result in an error if the agent goes into another idle task directly after
            if I:
                print('Preventing agent %s from being assigned to multiple tasks.'% I.agent_id)
                I().accepting_new_tasks = False

        elif ST == 'closest_node':
            # Find closet node in list
            new_list = {n: self.dist(agent, n, agent.location()) for n in list}
            I = self.get_dist(new_list)

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
        AExcl = [a for _id, a in self.AllAgentsList.items() if (_id is not agent.agent_id)]

        # Get blocked nodes
        occ = self.routing_manager.planner.load_occupied_nodes(ret=True)
        occupied = sum([v for k,v in occ.items() if k is not agent.agent_id], [])

        # Include navigation targets
        occupied += [a().interaction.response for a in AExcl if a().interaction and a().interaction.response and a.map_handler.is_node(a().interaction.response)]
        occupied += [a.goal() for a in AExcl if a.goal()]
        return occupied

    def get_dist(self, dist_list):
        if dist_list:
            return min(dist_list, key=dist_list.get)
        return None

    def dist(self, agent, start, goal):
        # Return the total distance from the start point to each goal
        try:
            if not type(goal) == list:
                goal = [goal]
            return sum([agent.map_handler.get_route_length(agent, start, g) for g in goal])
        except:
            print("Try-Except in manager.py for Action_Management modules")
            print(traceback.format_exc())
            return 0.0
