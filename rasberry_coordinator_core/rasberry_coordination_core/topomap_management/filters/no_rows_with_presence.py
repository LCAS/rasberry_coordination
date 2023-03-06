from rasberry_coordination.topomap_managem,ent.filters import utils


def filter(agent, agent_list):
    """ remove occupied rows """
    A = [a for a in agent_list
           if (a is not agent)
           and (a.navigation_properties["has_presence"])
           and (a.location()[0] == "r")
           and ("-c" in a.location) ]

    nodes_to_block = []
    for a in A:
        loc = a.loc.replace("r","").replace("c","")
        r,c = loc.split("-")    #r4.5-c7
        if "." in r:
            #is short robot
            m,s = r.split(".")
            if m == "7":
                block_row(int(m))
            elif m == "5":
                block_row(int(m))
                block_row(int(m)+1)
            elif m == "3":
                block_row(int(m)+1)
            block_row(r)


        else:
            # is tall robot
            ri = int(r)
            block_row(ri)    #r4

            #check previous rows
            if is_row(ri-0.5):
                block_row(ri-0.5) #r3.5
                block_row(ri-1)    #r3
            else:
                block_row(ri-0.3)  #r3.7

            #check next rows
            if is_row(ri+0.5):
                block_row(ri+0.5)    #r4.5
                block_row(ri+1)    #r5
            else:
                block_row(r+0.3)    #r4.3

