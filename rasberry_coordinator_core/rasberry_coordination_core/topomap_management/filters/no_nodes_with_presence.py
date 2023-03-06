from rasberry_coordination.topomap_management.filters import utils


def filter(agent, agent_list=[]):
    """ Remove all nodes containing agents which arent the given agent """
    ocupied_nodes = [a.location() for a in agent_list]   #<-- so where exactly are we supposed to be getting the agent list from???
    utils.block_nodes(occupied_nodes)
