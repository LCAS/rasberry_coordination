#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination.route_planners.fragment_planner import FragmentPlanner


class RouteFinder(object):  # TODO: investigate use of static class (return planner obj from __init__ instead of self)
    def __init__(self, planning_type, agents, callbacks):
        self.planning_type = planning_type
        self.agents = agents
        self.callbacks = callbacks

        planning_types = {'fragment_planner': self.fragment_planner,
                          'alternative_planner': self.alternative_planner}

        self.planner = planning_types[self.planning_type]()

    def find_routes(self):
        self.planner.find_routes()

    def fragment_planner(self):  # TODO: add direct object creation in __init__
        return FragmentPlanner(self.agents, self.callbacks)

    def alternative_planner(self):
        print('placeholder in ' + str(self.__class__))
        # return alternative_planner(self.agents, self.callbacks)
