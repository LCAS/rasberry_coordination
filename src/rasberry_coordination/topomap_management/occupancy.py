from rasberry_coordination.coordinator_tools import logmsg
from math import floor, ceil
from topological_navigation.tmap_utils import get_node_from_tmap2 as GetNode

class OccupancyFilters(object):

    @classmethod
    def exists(cls, row, node_list):
        return any([n for n in node_list if n.startswith("r%s-c"%row)])

    @classmethod
    def self(cls, map, node_list, node):
        return [node]

    @classmethod
    def entry(cls, map, node_list, node):

        #find entry node (all edges to node)
        #loop through edges till reaching a node with more than 1 new edge
        for i in range(1000):

            #Get list of ins and outs exclusing source
            edges = [e['node'] for e in GetNode(map, node)['node']['edges'] if e['node'] != node]

            #If we at a junction, break here
            if len(edges) > 1: break

            #Set next edge in line to check
            node = edges[0]

        return [node]

    @classmethod
    def neighbour_tall_ends(cls, map, node_list, node):
        #find ends of neghbouring short row
        options = cls.neighbour_row_tall(map, node_list, node)
        return [o for o in options if o.endswith('-cb') or o.endswith('-cy')]

    @classmethod
    def neighbour_row_tall(cls, map, node_list, node, sep=False):
        #########################################################
        # find all nodes which are in the adjecent tall row
        # row        r      f/c      ==+-     c-.3     goal
        # r0.7-ca -> 0.7 -> [0,1] -> [0,1] -> [ ,1] -> [ ,1]
        # r1-ca   -> 1   -> [1,1] -> [0,2] -> [ ,2] -> [ ,2]
        # r1.5-ca -> 1.5 -> [1,2] -> [1,2] -> [1,2] -> [1,2]
        # r2-ca   -> 2   -> [2,2] -> [1,3] -> [1,3] -> [1,3]
        # r2.5-ca -> 2.5 -> [2,3] -> [2,3] -> [2,3] -> [2,3]
        # r3-ca   -> 3   -> [3,3] -> [2,4] -> [2,4] -> [2,4]
        # r3.5-ca -> 3.5 -> [3,4] -> [3,4] -> [3,4] -> [3,4]
        # r4-ca   -> 4   -> [4,4] -> [3,5] -> [3,5] -> [3,5]
        # r4.5-ca -> 4.5 -> [4,5] -> [4,5] -> [4,5] -> [4,5]
        # r5-ca   -> 5   -> [5,5] -> [4,6] -> [4, ] -> [4, ]
        # r5.3-ca -> 5.3 -> [5,6] -> [5,6] -> [5, ] -> [5, ]
        # r5.7-ca -> 5.7 -> [5,6] -> [5,6] -> [ ,6] -> [ ,6]
        #########################################################

        # identify id
        row = node[1:].split('-c')[0]
        r = float(row)

        # split directions
        f = int(floor(r))
        c = int(ceil(r))

        # shift same
        d = f-1 if f==c else f
        u = c+1 if f==c else c

        # clear invalid
        d = 'None' if ('.7' in row) else d
        u = 'None' if ('.3' in row) else u

        # return list
        return [n for n in node_list if n.startswith("r%s-c"%d) or n.startswith("r%s-c"%u)]

    @classmethod
    def neighbour_short_ends(cls, map, node_list,  node):
        #find ends of neghbouring short row
        options = cls.neighbour_row_short(map, node_list, node)
        return [o for o in options if o.endswith('-cb') or o.endswith('-cy')]

    @classmethod
    def neighbour_row_short(cls, map, node_list, node):
        #########################################################
        # find all nodes which are in the adjecent short row
        # row        r       f,c     f-1, c     r==.x    +0.5          goal
        # r0.7-ca -> 0.7 -> [0,1] -> [-1, 1] -> [ ,1] -> [   , 1.5] -> [   , 1.5]
        # r1-ca   -> 1   -> [1,1] -> [ 0, 1] -> [0,1] -> [0.5, 1.5] -> [0.7, 1.5]
        # r1.5-ca -> 1.5 -> [1,2] -> [ 0, 2] -> [0,2] -> [0.5, 2.5] -> [0.7, 2.5]
        # r2-ca   -> 2   -> [2,2] -> [ 1, 2] -> [1,2] -> [1.5, 2.5] -> [1.5, 2.5]
        # r2.5-ca -> 2.5 -> [2,3] -> [ 1, 3] -> [1,3] -> [1.5, 3.5] -> [1.5, 3.5]
        # r3-ca   -> 3   -> [3,3] -> [ 2, 3] -> [2,3] -> [2.5, 3.5] -> [2.5, 3.5]
        # r3.5-ca -> 3.5 -> [3,4] -> [ 2, 4] -> [2,4] -> [2.5, 4.5] -> [2.5, 4.5]
        # r4-ca   -> 4   -> [4,4] -> [ 3, 4] -> [3,4] -> [3.5, 4.5] -> [3.5, 4.5]
        # r4.5-ca -> 4.5 -> [4,5] -> [ 3, 5] -> [3,5] -> [3.5, 5.5] -> [3.5, 5.3]
        # r5-ca   -> 5   -> [5,5] -> [ 4, 5] -> [4,5] -> [4.5, 5.5] -> [4.5, 5.3]
        # r5.3-ca -> 5.3 -> [5,6] -> [ 4, 6] -> [4, ] -> [4.5,    ] -> [4.5,    ]
        # r5.7-ca -> 5.7 -> [5,6] -> [ 4, 6] -> [ ,6] -> [   , 6.5] -> [   , 6.5]
        #########################################################

        # identify id
        row = node[1:].split('-c')[0]
        r = float(row)

        # split directions
        f = int(floor(r))
        c = int(ceil(r))

        # shift f
        d = f-1
        u = c

        # clear invalid
        d = 'None' if '.7' in row else d
        u = 'None' if '.3' in row else u

        # add decimal
        if d != 'None': d = d+0.7 if cls.exists(d+0.7, node_list) else d+0.5
        if u != 'None': u = u+0.3 if cls.exists(u+0.3, node_list) else u+0.5

        # return list
        return [n for n in node_list if n.startswith("r%s-c"%d) or n.startswith("r%s-c"%u)]
