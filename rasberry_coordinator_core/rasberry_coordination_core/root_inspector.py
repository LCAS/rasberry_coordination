#! /usr/bin/env python3
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os

from rasberry_coordination_core.srv import String as StringRequest
from rasberry_coordination_core.srv import StringResponse as StringResponse

class RootInspector(object):

    def __init__(self, topic, root):
        self.root = root

        global Service
        Service(topic, StringRequest, self.root_inspector_srv)

    def root_inspector_srv(self, req):
        resp = StringResponse()
        resp.success = True

        root = req.data
        d = root.split('.')[1:]
        tree = self.root
        for k in d:
            k = k.replace(']', '').split('[')

            if k[0] not in tree.__dict__: tree = tree.__dict__; break
            #Error processing request: 'dict' object has no attribute '__dict__'

            tree = tree.__getattribute__(k[0])
            if len(k) > 1:
                if type(tree) == type([]):
                    tree = tree[int(k[1])]
                elif type(tree) == type(dict()):
                    tree = tree[k[1]]
        resp.msg = str(tree)
        return resp
