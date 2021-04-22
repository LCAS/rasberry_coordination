#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination.route_planners.fragment_planner import FragmentPlanner


class RouteFinder(object):  # TODO: investigate use of static class (return planner obj from __init__ instead of self)
    def __init__(self, planning_type, robots, pickers, callbacks):
        """ Class to create a route_planner object and manage finding of routes (in both empty and congested maps)

        :param planning_type: the planning framework to use (eg. str:"fragment_planner")
        :param robots: pointer to robot_manager
        :param pickers: pointer to picker_manager
        :param callbacks: list of callbacks for direct access to functionality within the coordinator
        """
        self.planning_type = planning_type
        self.robot_manager = robots
        self.picker_manager = pickers
        self.callbacks = callbacks

        planning_types = {'fragment_planner': self.fragment_planner,
                          'alternative_planner': self.alternative_planner}
        self.planner = planning_types[self.planning_type]()

    def find_routes(self):
        """ Proxy function to self.planner.find_routes()

        :return: None
        """
        try:
            self.planner.find_routes()
        except AttributeError as e:
            print(e)
            logmsg(level="error", category="robot", msg='error w/ search_route(start_node, goal_node)')


    def fragment_planner(self):  # TODO: add direct object creation in __init__
        """ Create a FragmentPlanner object and populate it with the pointers to the agent managers and the callbacks

        :return: FragmentPlanner
        """
        return FragmentPlanner(self.robot_manager, self.picker_manager, self.callbacks)

    def alternative_planner(self):
        """ Example function to show how planning_types dict can be expanded in __init__
        Remove once second planner is added.

        :return: None
        """
        print('placeholder for ' + self.planning_type)
        # return alternative_planner(self.robot_manager, self.picker_manager)
