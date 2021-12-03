#! /usr/bin/env python
# -*- coding: utf-8 -*-


class Module(object):

    def __init__(self, module, actions):
        self.module = module
        self.actions = [[a, False] for a in actions]

    def repr(self):
        h = self.make_hash
        head = "%s [ %s ]"%(self.module, ' '.join([a[0] for a in self.actions]))
        deet = "%s   %s  "%(h(self.module,' '), ' '.join([h(a[0],['-','▀'][a[1]]) for a in self.actions]))
        return (head, deet)

    def make_hash(self, s, t): return len(s)*t

    def __call__(self, item):
        self.actions[item][1] = True


class Container(object):
    def repr(self):
        n=self.name
        X=[m.repr() for m in self.modules]

        head="%s: %s"%(n, ' '.join([x[0] for x in X]))
        deet="%s  %s"%(len(n)*' ', ' '.join([x[1] for x in X]))

        print(head)
        print(deet)

    def __init__(self,name):
        self.name=name
        self.modules = []

    def add(self, module, actions):
        self.modules.append(Module(module, actions))

    def set(self, module_id, action_id):
        self[module_id][action_id][1] = True

    def setL(self, items):
        [self.set(i[0],i[1]) for i in items]

    def __getitem__(self, item):
        return self.modules[item]


class Containers(object):
    def __init__(self):
        self.containers = []

    def add(self, container):
        self.containers.append(container)

    def __getitem__(self, item):
        return self.containers[item]

    def repr(self):
        for c in self.containers:
            c.repr()
            print('')


C = Containers()


c = Container("TOC")
c.add("agent",       ["pause", "resume", "cancel"])
c.add("task",        ["pause", "resume", "cancel"])
c.add("coordinator", ["pause", "resume", "cancel"])
C.add(c)

c = Container("CAR")
c.add("picker", ["call", "load", "cancel"])
C.add(c)

c = Container("TASKS(0)")
c.add("base", ["robot-init", "robot-idle", "human-init", "human-idle"])
C.add(c)

c = Container("TASKS(1)")
c.add("logistics_transportation", ["go-to-picker", "go-to-storage"])
C.add(c)

c = Container("TASKS(2)")
c.add("health_monitoring", ["low-battery-resp"])
C.add(c)

c = Container("TASKS(3)")
c.add("uv_treatment", ["edge-action", "row-action", "tunnel-action"])
C.add(c)

c = Container("TASKS(4)")
c.add("data_monitoring", ["edge-action", "row-action", "tunnel-action"])
C.add(c)

C[0][0](0)
# C[0][0](1)
# C[0][0](2)
# C[0][1](0)
# C[0][1](1)
C[0][1](2)
C[0][2](0)
C[0][2](1)
# C[0][2](2)

C[1][0](0)
C[1][0](1)
# C[1][0](2)

C[2][0](0)
C[2][0](1)
C[2][0](2)
C[2][0](3)

C[3][0](0)
C[3][0](1)

# C[4][0](0)
# C[4][0](1)
# C[4][0](2)

C[5][0](0)
# C[5][0](1)
# C[5][0](2)

C.repr()

