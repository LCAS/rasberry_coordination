#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination.agent_managers.robot_manager import RobotManager
from rasberry_coordination.agent_managers.picker_manager import PickerManager

from rasberry_coordination.route_planners.fragment_planner import FragmentPlanner

class RouteFinder(object):
        def __init__(self, planning_type, robots, pickers, callbacks):
            self.planning_type = planning_type               #could do static class or pre_init?
            self.robot_manager = robots
            self.picker_manager = pickers
            self.callbacks = callbacks

            planning_types = {'fragment_planner':self.fragment_planner,
                              'crh_star':self.crh_star}

            planning_types[self.planning_type]()


        def find_routes(self):
            self.planner.find_routes()

        def fragment_planner(self):
            self.planner = FragmentPlanner(self.robot_manager, self.picker_manager, self.callbacks)

        def crh_star(self):
            print('TODO:')
            # self.planner = crh_star_planner(self.robot_manager, self.picker_manager)

