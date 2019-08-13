#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import actionlib
import rospy

import topological_navigation.msg
import strands_navigation_msgs.msg


class Robot(object):
    """Robot class to wrap all ros interfaces to the physical/simulated robot
    """
    def __init__(self, robot_id, ):
        """initialise the Robot class

        Keyword arguments:

        robot_id - id of robot
        """
        self.robot_id = robot_id
        self.ns = "/%s/" %(robot_id)

        self.goal_node = "none"
        self.start_time = rospy.get_rostime()
        self.toponav_goal = topological_navigation.msg.GotoNodeGoal()
        self.toponav_result = None
        self.toponav_status = None
        self.toponav_route = None

        self.execpolicy_goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()
        self.execpolicy_result = None
        self.execpolicy_status = None
        self.execpolicy_current_wp = None

        # topological navigation action client
        self._topo_nav = actionlib.SimpleActionClient(self.ns + "topological_navigation", topological_navigation.msg.GotoNodeAction)

        # execute policy action client
        self._exec_policy = actionlib.SimpleActionClient(self.ns + "topological_navigation/execute_policy_mode", strands_navigation_msgs.msg.ExecutePolicyModeAction)

    def set_toponav_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """send_goal to topo_nav action client
        """
        rospy.loginfo("robot-%s has goal %s", self.robot_id, goal.target)
        if done_cb is None:
            done_cb = self._done_toponav_cb
        if feedback_cb is None:
            feedback_cb = self._fb_toponav_cb

        self.toponav_goal = goal
        self.toponav_result = None
        self.toponav_route = None
        self.toponav_status = None
        self._topo_nav.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

#        self._topo_nav.wait_for_result()

    def _fb_toponav_cb(self, fb):
        """feedback callback
        """
        self.toponav_route = fb.route

    def _done_toponav_cb(self, status, result):
        """done callback
        """
        self.toponav_goal = topological_navigation.msg.GotoNodeGoal()
        self.toponav_status = status
        self.toponav_result = result

    def cancel_toponav_goal(self, ):
        """
        """
        self._topo_nav.cancel_all_goals()
        self.toponav_goal = topological_navigation.msg.GotoNodeGoal()
        self.toponav_result = None
        self.toponav_route = None
        self.toponav_status = None

    def set_execpolicy_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """send_goal to execute_policy_mode action client
        """
        rospy.loginfo("robot-%s has an edge_policy goal", self.robot_id)
        rospy.loginfo(goal)
        if done_cb is None:
            done_cb = self._done_execpolicy_cb
        if feedback_cb is None:
            feedback_cb = self._fb_execpolicy_cb

        self.execpolicy_goal = goal
        self.execpolicy_current_wp = None
        self.execpolicy_result = None
        self.execpolicy_status = None
        self._exec_policy.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
#        self._exec_policy.wait_for_result()

    def _fb_execpolicy_cb(self, fb):
        """feedback callback
        """
        self.execpolicy_current_wp = fb.current_wp
        self.execpolicy_status = fb.status

    def _done_execpolicy_cb(self, status, result):
        """done callback
        """
        self.execpolicy_status = status
        self.execpolicy_result = result
        self.execpolicy_goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()

    def cancel_execpolicy_goal(self, ):
        """
        """
        self._exec_policy.cancel_all_goals()
        self.execpolicy_goal = strands_navigation_msgs.msg.ExecutePolicyModeGoal()
        self.execpolicy_current_wp = None
        self.execpolicy_result = None
        self.execpolicy_status = None

