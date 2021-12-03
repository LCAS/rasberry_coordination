#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date: 24/Nov/2021
# ----------------------------------

from random import randint as r_int, choice as r_lst


class TestModule(object):

    def __init__(self):
        self.N = range(100)
        self.minT = 0
        self.maxT = 60
        self.actions = ["picker01-INIT","picker01-AWAITING", "picker01-"]
        self.generate_action_sequence()
        rospy.set_param('rob_mov_dlay', 0)
        self.execute_action_sequence()

    def generate_action_sequence(self):
        self.sequence = [[r_lst(self.actions), r_int(self.minT, self.maxT)] for i in self.N]

    def execute_action_sequence(self): pass

    class virtual_picker(self):
        to_coordinator = ['CALLED', 'LOADED', 'INIT']
        from_coordinator = ['ACCEPT', 'ARRIVED', 'INIT']



    class virtual_toc(self):
        to_coordinator = ['']
        from_coordinator = ['']
        format = Message()

        def __init__(self):
            self.generate_all_publish_permutations()

        def generate_all_publish_permutations(self):
            self.permutations = None

        def active_task_list_sub(self, msg): self.active_tasks = msg

if __name__ == '__main__':
    tester = TestModule()

















