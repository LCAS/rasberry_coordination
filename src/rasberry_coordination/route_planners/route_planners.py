#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

from rasberry_coordination.agent_managers.robot_manager import RobotManager
from rasberry_coordination.agent_managers.picker_manager import PickerManager

from rasberry_coordination.route_planners.fragment_planner import FragmentPlanner


class RouteFinder(object):  # TODO: investigate use of static class (return planner obj from __init__ instead of self)
        def __init__(self, planning_type, robots, pickers, callbacks):
            self.planning_type = planning_type
            self.robot_manager = robots
            self.picker_manager = pickers
            self.callbacks = callbacks

            planning_types = {'fragment_planner': self.fragment_planner,
                              'alternative_planner': self.alternative_planner}
            self.planner = planning_types[self.planning_type]()

        def find_routes(self):
            self.planner.find_routes()

        def fragment_planner(self):  # TODO: add direct object creation in __init__
            return FragmentPlanner(self.robot_manager, self.picker_manager, self.callbacks)

        def alternative_planner(self):
            print('placeholder')
            # return alternative_planner(self.robot_manager, self.picker_manager)

