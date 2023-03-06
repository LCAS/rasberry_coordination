def block_nodes(cls, agent, node_list):
    """remove incoming edges to the nodes in a given list (essentially isolating nodes from the map)"""
    for node in agent.map_handler.filtered_map["nodes"]:
        node["node"]["edges"] = [e for e in node["node"]["edges"] if e not in node_list]


def unblock_node(cls, agent, node_to_unblock):
    """ unblock a node by adding details from a backup of the unfiltered map """
    nodes_to_append = []
    edges_to_append = []

    # get list of nodes and edges toi copy to filtered map
    for node in agent.map_handler.empty_map["nodes"]:
        for edge in node["node"]["edges"]:
            if edge["node"] == node_to_unblock:
                nodes_to_append.append(node["node"]["name"])
                edges_to_append.append(edge)

    # apply nodes and edges to filtered map
    for node in agent.map_handler.filtered_map["nodes"]:
        if node["node"]["name"] in nodes_to_append:
            ind_to_append = nodes_to_append.index(node["node"]["name"])
            node["node"]["edges"].append(edges_to_append[ind_to_append])


def block_prefix(agent, node_prefix):
    """ block all nodes which match the given format  """
    nodes_of_interest = [node["node"]["name"] for node in agent.map_handler.filtered_map["nodes"] if node["node"]["name"].startswith(node_prefix)]
    cls.block_nodes(nodes_of_interest)
