#!/usr/bin/env python
import rospy, yaml, json
from std_msgs.msg import String as Str
from topological_navigation_msgs.msg import TopologicalMap as TMap
from pprint import pprint


class Restrictor():
    def __init__(self, initial_restriction, enable_eval_sub):
        self.tmap = None
        self.eval = Str(initial_restriction) if initial_restriction else None
        self.res_tmap_pub_2 = rospy.Publisher('/restricted_topological_map_2', Str, queue_size=2, latch=True)
        if enable_eval_sub: self.res_eval_sub = rospy.Subscriber('/restriction_evalator', Str, self.py_eval)
        self.tmap_sub_2 = rospy.Subscriber('/topological_map_2', Str, self.tmap_cb)

    def tmap_cb(self, msg):
        self.tmap = yaml.safe_load(msg.data)
        if self.eval: self.py_eval(self.eval)

    def py_eval(self, msg):
        """ This eval is compared with the restriction condition for the topological map, only those which match the evaluation remain.
        e.g. | node.restriction = "['short', 'tall']"
             | msg.data = "'short' in $"
             | eval( "'short' in $".replace('$', node.restriction) )
             | eval( "'short' in ['short', 'tall']" ) = True
        """
        rospy.loginfo("py_eval: %s"%msg.data)
        if not self.tmap:
            self.eval = msg
            rospy.logwarn("No TMap yet, will apply eval once map arrives.")
            return
        tmap = self.tmap
        nodes_to_keep = []

        condition = msg.data

        # Quick pass for duplicate check conditions
        condition_results = {True:[], False:[]}
        def check_outcome(query, restriction):
            check = query.replace('$', str(restriction))
            if check in condition_results[True]:  return True
            if check in condition_results[False]: return False
            result = eval(check)
            condition_results[result] += [check]
            return result


        # Filter nodes
        tmap['nodes'] = [n for n in tmap['nodes'] if check_outcome(condition, n['node']['restrictions_planning'])]

        # Filter edges
        kept_nodes = [n['node']['name'] for n in tmap['nodes']]
        for node in tmap['nodes']:
            if 'edges' in node['node']:
                node['node']['edges'] = [e for e in node['node']['edges'] if e['node'] in kept_nodes]
                node['node']['edges'] = [e for e in node['node']['edges'] if check_outcome(condition, e['restrictions_planning'])]

        pprint(condition_results)
        print(len(self.tmap['nodes']))
        print(len(tmap['nodes']))

        # Publish new map
        self.tmap = tmap
        #self.res_tmap_pub_1.publish(self.tmap)
        s = json.dumps(self.tmap)
        self.res_tmap_pub_2.publish(Str(s))

if __name__=='__main__':
    rospy.init_node('restrictor')
    ir = rospy.get_param("~initial_restriction", "")
    es = rospy.get_param("~enable_eval_sub", True)
    rospy.loginfo('Restrictor launched with initial restriction as: %s'%ir)
    r = Restrictor(initial_restriction=ir, enable_eval_sub=es)
    rospy.spin()
