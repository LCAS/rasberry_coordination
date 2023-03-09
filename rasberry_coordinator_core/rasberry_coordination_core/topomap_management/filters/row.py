from rasberry_coordination_core.topomap_management.filters import utils

def filter(agent, agent_list=[]):
    """ block access to all rows based on agent type conditions """
    pass

#def block_rows(cls, agent, occupied_nodes):
#    """ block access to each row if the row is occupied """
#    #This only works on the assumption that small rows are staggered left of the tall rows
#    for node in occupied_nodes:
#        name = node["name"]
#        if name.startswith('tall_t'):
#            #lock: [tall, short, TALL, short, tall]
#            _,t,r,_ = name.split('_')
#            #lock tall rows:
#            cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])-1 ))
#            cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])   ))
#            cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])+1 ))
#            #lock short rows:
#            cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])   ))
#            cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])+1 ))
#            pass
#        elif name.startswith('small_t'):
#            #lock: [tall, SHORT, tall]
#            _,t,r,_ = name.split('_')
#            #lock short rows:
#            cls.block_row_ends(agent, 'small_%s_r%i' % ( t , int(r[1:])   ))
#            #lock tall rows:
#            cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])-1 ))
#            cls.block_row_ends(agent, 'tall_%s_r%i' % ( t , int(r[1:])   ))
#            pass
#        else:
#            pass


#def block_row_ends(cls, agent, row_name='small_t1_r2'):
#    #Identify all nodes which are related to the row_name given
#    nodes_of_interest = [int(n.split('_c')[1]) for n in agent.map_handler.empty_node_list if n.startswith(row_name)]
#    if not nodes_of_interest: return
#
#    #Identify the full names of the ends of the row
#    row_start = '%s_c%i' % (row_start_id, min(nodes_of_interest))
#    row_end = '%s_c%i' % (row_start_id, max(nodes_of_interest))
#
#    #Block the ends of the rows being traversed
#    cls.block_nodes(agent, [row_start, row_end])


#def unblock_parent_row(cls, agent, node_name):
#    #Exit early if the node is not in a row
#    if node_name.split('_')[0] not in ['tall_t', 'small_t']: return
#
#    #Identify the parent row
#    row_name = node_name.split('_c')[0]
#
#    #Identify all nodes which are related to the row_name given
#    nodes_of_interest = [int(n.split('_c')[1]) for n in agent.map_handler.empty_node_list if n.startswith(row_name)]
#    if not nodes_of_interest: return
#
#    #Identify the full names of the ends of the row
#    row_start = '%s_c%i' % (row_start_id, min(nodes_of_interest))
#    row_end = '%s_c%i' % (row_start_id, max(nodes_of_interest))
#
#    #Block the ends of the rows being traversed
#    cls.unblock_node(agent, row_start)
#    cls.unblock_node(agent, row_end)
