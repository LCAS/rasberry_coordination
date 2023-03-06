class FragmentPlanner_map_filter(object):

    @classmethod
    def generate_filtered_map(cls, agent, start_node, goal_node, occupied_nodes):
        agent.map_handler.start_map_reset()

        #Block access to nodes occupied by other agents
        cls.block_nodes(agent, occupied_nodes)
        cls.unblock_node(agent, start_node)
        cls.unblock_node(agent, goal_node) # not strictly necesary as pickers and storage agents do not use presence and thus will not be in occupied_nodes

        agent.map_handler.complete_map_reset()

    @classmethod
    def block_nodes(cls, agent, occupied_nodes):
        """remove incoming edges to the list of agent nodes in the available_tmap
        and update the available_route_search object with the new map
        :param agent_nodes: list of nodes occupied by other agents, list
        """

        # # Nothing to do if restrictions are not used
        # if 'restrictions' not in agent.navigation_properties: return

        ocn = occupied_nodes
        # if agent.location() in occupied_nodes: ocn.remove(agent.location())

        # for node in agent.map_handler.filtered_map["nodes"]:
        #     to_pop = []
        #     for i in range(len(node["node"]["edges"])):
        #         if node["node"]["edges"][i]["node"] in ocn:
        #             to_pop.append(i)
        #     if to_pop:
        #         to_pop.reverse()
        #         for j in to_pop:
        #             node["node"]["edges"].pop(j)

        for node in agent.map_handler.filtered_map["nodes"]:
            node["node"]["edges"] = [e for e in node["node"]["edges"] if e not in ocn]

    @classmethod
    def unblock_node(cls, agent, node_to_unblock):
        """ unblock a node by adding details from unfiltered map """
        nodes_to_append = []
        edges_to_append = []

        # for each edge in network, if edge connects to a node to unblock, add to list
        for node in agent.map_handler.empty_map["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["node"] == node_to_unblock:
                    nodes_to_append.append(node["node"]["name"])
                    edges_to_append.append(edge)

        # for each node in empty map, if node is to be unblocked, add a extra edge
        for node in agent.map_handler.filtered_map["nodes"]:
            if node["node"]["name"] in nodes_to_append:
                ind_to_append = nodes_to_append.index(node["node"]["name"])
                node["node"]["edges"].append(edges_to_append[ind_to_append])

    @classmethod
    def block_rows(cls, agent, occupied_nodes):
        """ block access to each row if the row is occupied """
        #This only works on the assumption that small rows are staggered left of the tall rows
        for node in occupied_nodes:
            name = node["name"]
            if name.startswith('tall_t'):
                #lock: [tall, short, TALL, short, tall]
                _,t,r,_ = name.split('_')
                #lock tall rows:
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])-1 ))
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])   ))
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])+1 ))
                #lock short rows:
                cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])   ))
                cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])+1 ))
                pass
            elif name.startswith('small_t'):
                #lock: [tall, SHORT, tall]
                _,t,r,_ = name.split('_')
                #lock short rows:
                cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])   ))
                #lock tall rows:
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])-1 ))
                cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])   ))
                pass
            else:
                pass

    @classmethod
    def block_row_ends(cls, agent, row_name='small_t1_r2'):
        #Identify all nodes which are related to the row_name given
        nodes_of_interest = [int(n.split('_c')[1]) for n in agent.map_handler.empty_node_list if n.startswith(row_name)]
        if not nodes_of_interest: return

        #Identify the full names of the ends of the row
        row_start = '%s_c%i' % (row_start_id, min(nodes_of_interest))
        row_end = '%s_c%i' % (row_start_id, max(nodes_of_interest))

        #Block the ends of the rows being traversed
        cls.block_nodes(agent, [row_start, row_end])

    @classmethod
    def unblock_parent_row(cls, agent, node_name):
        #Exit early if the node is not in a row
        if node_name.split('_')[0] not in ['tall_t', 'small_t']: return

        #Identify the parent row
        row_name = node_name.split('_c')[0]
        
        #Identify all nodes which are related to the row_name given
        nodes_of_interest = [int(n.split('_c')[1]) for n in agent.map_handler.empty_node_list if n.startswith(row_name)]
        if not nodes_of_interest: return

        #Identify the full names of the ends of the row
        row_start = '%s_c%i' % (row_start_id, min(nodes_of_interest))
        row_end = '%s_c%i' % (row_start_id, max(nodes_of_interest))

        #Block the ends of the rows being traversed
        cls.unblock_node(agent, row_start)
        cls.unblock_node(agent, row_end)

















