class ActionResponses():

    """ Services offerd by Coordinator to assist with tasks """
    def offer_service(self, agent):

        #Identify action properties
        action_type =  agent().action['action_type']
        action_style = agent().action['action_style'] if 'action_style' in agent().action else ''
        action_deets = {k:v for k,v in agent().action.items() if k not in ['action_type', 'action_style', 'response_location']}

        #Define resposse functions
        responses = {'find_agent':self.find_agent,
                     'find_node': self.find_node,
                     'send_info': self.send_info}

        #Perform Action
        agent().action['response_location'] = responses[action_type](agent)
        if agent().action['response_location'] == 'sent': return
        if agent().action['response_location']:
            agent().action_required = False

    """ Action Category """
    def find_agent(self, agent):
        action_style = agent().action['action_style']

        if 'list' in agent().action:
            agent_list = agent().action['list']
            A = {agent_id:self.AllAgentsList[agent_id] for agent_id in agent_list}
        else:
            agent_type = agent().action['agent_type']
            A = {a.agent_id:a for a in self.AllAgentsList.values() if (a is not agent) and (agent_type in a.roles())}

        responses = {"closest": self.find_closest_agent}
        return responses[action_style](agent, A)

    def find_node(self, agent):
        action_style = agent().action['action_style']

        if 'descriptor' in agent().action:
            if agent().action['action_style'] in ["row_ends", "rows"]:
                N = agent().action['descriptor']
            else:
                descriptor = agent().action['descriptor']
                occupied = get_occupied_nodes(agent)
                N = [n['id'] for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in occupied)]
        else:
            N = agent().action['list']

        responses = {"closest": self.find_closest_node,
                     "row_ends": self.find_row_ends,
                     "rows": self.find_rows}
        return responses[action_style](agent, N)

    """ Action Style """
    def find_closest_agent(self, agent, agent_list):
        dist_list = {a.agent_id:self.dist(a, a.location(), agent.location()) for a in agent_list.values() if a.registration and a.map_handler.is_node_restricted(start_location)}
        return agent_list[get_dist()]

    def find_closest_node(self, agent, node_list):
        dist_list = {n:self.dist(agent, n, agent.location()) for n in node_list}
        return get_dist()

    """ Tools """
    def find_occupied_nodes(self, agent):
        AExcl = [a for _id, a in self.AllAgentsList.items() if (_id is not agent.agent_id)]
        occupied = [a.location.current_node for a in AExcl if a.location.current_node]  # Check if node is occupied
        occupied += [a().action['response_location'] for a in AExcl if 'response_location' in a().action and a().action['response_location'] and a.map_handler.is_node(a().action[rl])]  # Include navigation targets
        occupied += [a.goal() for a in AExcl if a.goal()] # Include navigation targets
        return occupied
    def get_dist(self, dist_list):
        if dist_list:
            return min(dist_list, key=dist_list.get)
        return None
    def dist(self, agent, start_node, goal_node):
        return agent.map_handler.get_route_length(agent, start_node, goal_node)


    # def send_info(self, agent):
    #     pub1 = Publisher('/car_client/info/map', Str, queue_size=1, latch=True)
    #     pub1.publish(agent.map_handler.simplify())  # TODO: agent only holds restricted map, could this cause issues? (this only called by humans though, and they use full)
    #     pub2 = Publisher('/car_client/info/robots', Str, queue_size=1, latch=True)
    #     pub2.publish(self.agent_manager.simplify())
    #     return 'sent'
    # def find_row_ends(self, agent, row_id):
    #     return self.route_finder.planner.get_row_ends(agent, row_id)
    # def find_rows(self, agent, tunnel_id):
    #     return self.route_finder.planner.get_rows(agent, tunnel_id)






def offerservice(agent):
    action = agent.action

    if action['type'] == 'search':
        list = self.get_list(agent)
        item = self.get_item(agent, list)
        action.response = item
    elif action['type'] == 'info':
        resp = get_info()
        action.response = resp

def get_info(agent):
    action = agent().action
    FO = action.info

    if FO == 'send_info':
        pub1 = Publisher('/car_client/info/map', Str, queue_size=1, latch=True)
        pub1.publish(agent.map_handler.simplify())  # TODO: agent only holds restricted map, could this cause issues? (this only called by humans though, and they use full)
        pub2 = Publisher('/car_client/info/robots', Str, queue_size=1, latch=True)
        pub2.publish(self.agent_manager.simplify())
        return 'sent'

    elif FO == 'find_row_ends':
        return self.route_finder.planner.get_row_ends(agent, row_id)

    elif FO == 'find_rows':
        return self.route_finder.planner.get_rows(agent, tunnel_id)



def get_list(agent):
    action = agent().action
    GR = action.grouping

    if GR == 'node_list':
        L = action.list

    elif GR == 'agent_list':
        L = {agent_id: self.AllAgentsList[agent_id] for agent_id in action['list']}

    elif GR == 'node_descriptor':
        descriptor = agent().action['descriptor']
        occupied = get_occupied_nodes(agent)
        L = [n['id'] for n in self.special_nodes if (descriptor in n['descriptors']) and (n['id'] not in occupied)]

    elif GR == 'agent_descriptor':
        descriptor = agent().action['descriptor']
        L = {a.agent_id: a for a in self.AllAgentsList.values() if (a is not agent) and a.registration and (descriptor in a.roles()) }

    elif GR == 'new_list_generators_go_here':
        L = {}

    return L


def get_item(agent, list):
    action = agent().action
    location = agent.location()
    ST = action.style

    if ST == 'closest_agent':
        new_list = {k: self.dist(v, v.location(), location) for k,v in list.items()}
        I = agent_list[get_dist(new_list)]

    elif ST == 'closest_node':
        new_list = {n:self.dist(agent, n, agent.location()) for n in list}
        I = get_dist(new_list)

    elif ST == 'new_identifications_go_here':
        I = None

    return I



def get_dist(self, dist_list):
    if dist_list:
        return min(dist_list, key=dist_list.get)
    return None


def dist(self, agent, start_node, goal_node):
    return agent.map_handler.get_route_length(agent, start_node, goal_node)
