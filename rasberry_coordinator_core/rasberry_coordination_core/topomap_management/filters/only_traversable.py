from rasberry_coordination_core.topomap_management.filters import utils


def filter(agent, agent_list=[]):
    """ Remove nodes which are not traversable by the given agent """
    untraversible = [node["node"]["name"] for node in agent.map_manager.filtered_map["nodes"] if agent.naivgation_properties["restrictions"] not in node["node"]["filters"]]
    utils.block_nodes(untraversible)
