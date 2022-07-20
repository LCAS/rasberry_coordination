
from rasberry_coordination.topomap_management.filters import utils

def filter(agent, agent_list=[]):
   """ unblock the agent's current location, and navigation target """
   utils.unblock_node(agent.location())
   utils.unblock_node(agent().target)
