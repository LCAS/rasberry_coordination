#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import threading
import actionlib
import rospy

import geometry_msgs.msg
import std_msgs.msg
import std_srvs.srv
import topological_navigation.msg
import topological_navigation.tmap_utils
import topological_navigation.route_search
import rasberry_people_perception.topological_localiser

import rasberry_coordination.msg


class Robot(object):
    """Robot class to wrap all ros interfaces to the physical/simulated robot
    """
    def __init__(self, robot_id, local_storage, charging_node, base_station, unified=False):
        """initialise the Robot class

        Keyword arguments:

        robot_id - id of robot
        local_storage - local storage node in topological map
        charging_node - charging node in topological map
        """
        self.robot_id = robot_id
        if unified:
            self.ns = "/" # now working with only one robot and no clear ns differentiation
        else:
            self.ns = "/%s/" %(robot_id)
        self.robot_id = robot_id

        self.local_storage = local_storage # local storage node
        self.charging_node = charging_node # charging area node
        self.base_station = base_station # base station where the robot should wait

        self.topo_localiser = rasberry_people_perception.topological_localiser.TopologicalNavLoc()

        self.lock = threading.Lock()

        self.pose = geometry_msgs.msg.PoseStamped()
        self.pose.header.frame_id = "map"
        self._pose_sub = rospy.Subscriber(self.ns + "robot_pose", geometry_msgs.msg.Pose, self._update_pose_cb)

        self.closest_node = "none"
        # TODO: at the moment the topological navigation stack does n't respect name spacing and work with base ros_names
        self._closest_node_sub = rospy.Subscriber(self.ns + "closest_node", std_msgs.msg.String, self._update_closest_cb)

        # 0 - idle, 1 - transporting_to_picker, 2 - waiting for loading,
        # 3 - waiting for unloading, 4 - transporting to storage, 5- charging
        # 6 - return to base from storage
        self.states = {0:"Idle", 1:"Going to picker", 2:"Waiting for loading",
                       3:"Waiting for unloading", 4:"Going to storage",
                       5:"Charging", 6:"Going to base", 7:"Stuck"}
        self.mode = 0
        self.goal_node = "none"
        self.start_time = rospy.get_rostime()
        self.battery = 100. # TODO:

        self._cancelled = False
        self._preempted = False

        self.is_idle = lambda: True if self.mode==0 else False
        self.get_state = lambda: self.mode

        # topological navigation action client
        # TODO: at the moment the topological navigation stack does n't respect name spacing and work with base ros_names
        self._topo_nav = actionlib.SimpleActionClient(self.ns + "topological_navigation", topological_navigation.msg.GotoNodeAction)

        # tray loading and unloading services
        self.tray_loaded = False
        self.tray_unloaded = True

        self.tray_loaded_srv = rospy.Service(self.ns + "tray_loaded", std_srvs.srv.Trigger, self._tray_loaded_cb)
        self.tray_unloaded_srv = rospy.Service(self.ns + "tray_unloaded", std_srvs.srv.Trigger, self._tray_unloaded_cb)

        # assign robot action server and client
        self.collect_tray_action = actionlib.SimpleActionServer(self.ns + "collect_tray", rasberry_coordination.msg.CollectTrayAction, self._collect_tray_cb, False)
        self.collect_tray_action.register_preempt_callback(self._preempt_cb)
        self.collect_tray_action.start()

        self.collect_tray = actionlib.SimpleActionClient(self.ns + "collect_tray", rasberry_coordination.msg.CollectTrayAction)
        self._fb_msg = rasberry_coordination.msg.CollectTrayFeedback()

        self.task = None

    def get_state(self, ):
        """return the state of the robot, goal_node if any and the time at which
        the robot reached this mode
        """
        return (self.states[self.mode], self.goal_node, self.start_time)

    def _update_pose_cb(self, msg):
        """callback function to update robot_pose topics.
        it alse tries to update the closest_node if lock can be acquired
        """
        self.pose.pose = msg
        # TODO: check this is thread safe
        locked = self.lock.acquire(False)
        if locked:
            _, closeststr = self.topo_localiser.localise_pose(self.pose)
            if closeststr != "none":
                self.closest_node = closeststr
            self.lock.release()

    def _update_closest_cb(self, msg):
        """callback function to update closest_node topics.
        this is a latched topics and with multiple roscores, the updates may
        not be at an ideal rate which can cause problems for scheduling.
        so robot_pose subscriber callback also updates closest_node.
        """
        locked = self.lock.acquire(False)
        if locked:
            if msg.data != "none":
                self.closest_node = msg.data
            self.lock.release()

    def _tray_loaded_cb(self, req):
        """callback for tray_loaded service
        """
        self.tray_unloaded = False
        self.tray_loaded = True
        return (True, "")

    def _tray_unloaded_cb(self, req):
        """callback for tray_unloaded service
        """
        self.tray_loaded = False
        self.tray_unloaded = True
#        response = std_srvs.srv.Trigger()
#        success=True
#        response.message = ""
        return (True, "")

    def _go_to_base(self, ):
        """wrapper for sending specific goal (base_station) to topo_nav action client
        """
        rospy.loginfo("send robot-%s to base station", self.robot_id)
        self.mode = 6
        self.goal_node = self.base_station
        self.start_time = rospy.get_rostime()
        self._set_topo_nav_goal(goal_node=self.base_station)

    def _go_to_storage(self, ):
        """wrapper for sending specific goal (storage) to topo_nav action client
        """
        rospy.loginfo("send robot-%s to storage", self.robot_id)
        self.mode = 4
        self.goal_node = self.local_storage
        self.start_time = rospy.get_rostime()
        self._set_topo_nav_goal(goal_node=self.local_storage,
                               done_cb=self._to_storage_done_cb,
                               feedback_cb=self._fb_cb)

    def _go_to_picker(self, picker_node):
        """wrapper for sending specific goal (picker_node) to topo_nav action client
        """
        rospy.loginfo("send robot-%s to picker", self.robot_id)
        self.mode = 1
        self.goal_node = picker_node
        self.start_time = rospy.get_rostime()
        self._set_topo_nav_goal(goal_node=picker_node,
                               done_cb=self._to_picker_done_cb,
                               feedback_cb=self._fb_cb)

    def _set_topo_nav_goal(self, goal_node, done_cb=None, active_cb=None, feedback_cb=None):
        """send_goal and set feedback and done callbacks to topo_nav action client
        """
        goal = topological_navigation.msg.GotoNodeGoal()
        rospy.loginfo("robot-%s has goal %s", self.robot_id, goal_node)
        goal.target = goal_node
        goal.no_orientation = False
        self._topo_nav.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
        self._topo_nav.wait_for_result()

    def _fb_cb(self, fb):
        """feedback callback for
        """
        self._fb_msg.task_id = self.task.task_id
        self._fb_msg.mode = str(self.mode)
        self._fb_msg.closest_node = self.closest_node
        self._fb_msg.route = fb.route
        self.collect_tray_action.publish_feedback(self._fb_msg)

    def _to_storage_done_cb(self, status, result):
        """done callback for to_storage topo_nav action
        """
        if result.success:
            self._fb_msg.task_id = self.task.task_id
            self._fb_msg.mode = str(self.mode)
            self._fb_msg.closest_node = self.closest_node
            self._fb_msg.route = "reached storage"
            self.collect_tray_action.publish_feedback(self._fb_msg)
#            rospy.loginfo("robot-%s reached storage", self.robot_id)
        elif not rospy.is_shutdown():
            self._fb_msg.task_id = self.task.task_id
            self._fb_msg.mode = str(self.mode)
            self._fb_msg.closest_node = self.closest_node
            self._fb_msg.route = "failed to reach storage"
            self.collect_tray_action.publish_feedback(self._fb_msg)
#            rospy.loginfo("robot-%s failed to reach storage", self.robot_id)

    def _to_picker_done_cb(self, status, result):
        """done callback for to_picker topo_nav action
        """
        if result.success:
            self._fb_msg.task_id = self.task.task_id
            self._fb_msg.mode = str(self.mode)
            self._fb_msg.closest_node = self.closest_node
            self._fb_msg.route = "reached picker"
            self.collect_tray_action.publish_feedback(self._fb_msg)
#            rospy.loginfo("robot-%s reached picker", self.robot_id)
        elif not rospy.is_shutdown():
            self._fb_msg.task_id = self.task.task_id
            self._fb_msg.mode = str(self.mode)
            self._fb_msg.closest_node = self.closest_node
            self._fb_msg.route = "failed to reach the picker"
#            rospy.loginfo("root-%s failed to reach the picker", self.robot_id)

    def _wait_for_tray_load(self, min_load_duration, max_load_duration):
        """wait for loading a tray until tray_loaded is set or timeout
        """
        self.mode = 2 # wait for picker to load
        self.goal_node = "none"
        self.start_time = rospy.get_rostime()
        self.tray_loaded = False
        time_start = rospy.get_rostime()
        while not rospy.is_shutdown() and not self._cancelled and not self._preempted:
            time_delta = rospy.get_rostime() - time_start

            if self.tray_loaded:
                break
            elif time_delta >= max_load_duration:
                self.tray_loaded = True
                break
            else:
                self._fb_msg.task_id = self.task.task_id
                self._fb_msg.mode = str(self.mode)
                self._fb_msg.closest_node = self.closest_node
                self._fb_msg.route = "robot-%s loading now: remaining time max %d s" %(self.robot_id, time_delta.secs)
                self.collect_tray_action.publish_feedback(self._fb_msg)
            rospy.sleep(0.2)

        return self.tray_loaded

    def _wait_for_tray_unload(self, min_unload_duration, max_unload_duration):
        """wait for unloading a tray until tray_unloaded is set or timeout
        """
        self.mode = 3 # wait for someone to unload
        self.goal_node = "none"
        self.start_time = rospy.get_rostime()
        self.tray_unloaded = False
        time_start = rospy.get_rostime()
        while not rospy.is_shutdown() and not self._cancelled and not self._preempted:
            time_delta = rospy.get_rostime() - time_start
            if self.tray_unloaded:
                break
            elif time_delta >= max_unload_duration:
                self.tray_unloaded = True
                break
            else:
                self._fb_msg.task_id = self.task.task_id
                self._fb_msg.mode = str(self.mode)
                self._fb_msg.closest_node = self.closest_node
                self._fb_msg.route = "unloading now: remaining time max %d s" %(time_delta.secs)
                self.collect_tray_action.publish_feedback(self._fb_msg)
            rospy.sleep(1.)

        return self.tray_unloaded

    def _collect_tray_cb(self, goal):
        """execution callback for collect_tray
        """
        rospy.loginfo("robot-%s received CollectTray task-%d", self.robot_id, goal.task.task_id)
        self.task = goal.task

        _result = False
        reached_picker = False
        tray_loaded = False
        reached_storage = False
        tray_unloaded = False
        reached_base = False

        while not(self._cancelled or _result or rospy.is_shutdown()):

            if not reached_picker:
                # go to picker action
                self._go_to_picker(goal.task.start_node_id)
                if self._cancelled:
                    reached_picker = False
                else:
                    result = self._topo_nav.get_result()
                    if result is None:
                        reached_picker = False
                    else:
                        reached_picker = result.success

                if reached_picker:
                    rospy.loginfo("robot-%s reached the picker", self.robot_id)
                else:
                    self.publish_feedback("failed to reach the picker")
                    rospy.loginfo("robot-%s failed to reach the picker", self.robot_id)
                    break

            elif reached_picker and not tray_loaded:
                # wait for min(max_loading_duration, tray_loaded)
                tray_loaded = self._wait_for_tray_load(goal.min_load_duration, goal.max_load_duration)
                if self._cancelled:
                    tray_loaded = False

                if tray_loaded:
                    self.publish_feedback("tray loaded")
                    rospy.loginfo("robot-%s tray loaded", self.robot_id)
                elif not tray_loaded:
                    self.publish_feedback("failed to load tray")
                    rospy.loginfo("robot-%s failed to load tray", self.robot_id)
                    break

            elif tray_loaded and not reached_storage:
                # go to storage action
                self._go_to_storage()
                self._topo_nav.wait_for_result()
                if self._cancelled:
                    reached_storage = False
                else:
                    result = self._topo_nav.get_result()
                    if result is None:
                        reached_storage = False
                    else:
                        reached_storage = result.success

                if reached_storage:
                    self.publish_feedback("reached the storage")
                    rospy.loginfo("robot-%s reached the storage", self.robot_id)
                else:
                    self.publish_feedback("failed to reach the storage")
                    rospy.loginfo("robot-%s failed to reach the storage", self.robot_id)
#                    self._topo_nav.cancel_all_goals()
                    break

            elif reached_storage and not tray_unloaded:
                # wait for min(max_unloading_duration, tray_unloaded)
                tray_unloaded = self._wait_for_tray_unload(goal.min_unload_duration, goal.max_unload_duration)
                if self._cancelled:
                    tray_unloaded = False

                if tray_unloaded:
                    self.publish_feedback("tray unloaded")
                    rospy.loginfo("robot-%s tray unloaded", self.robot_id)
                elif not tray_unloaded:
                    self.publish_feedback("failed to unload tray")
                    rospy.loginfo("robot-%s failed to unload tray", self.robot_id)
                    break

            elif tray_unloaded and not reached_base:
                # go to base_station action
                self._go_to_base()
                self._topo_nav.wait_for_result()
                if self._cancelled:
                    reached_base = False
                else:
                    result = self._topo_nav.get_result()
                    if result is None:
                        reached_base = False
                    else:
                        reached_base = result.success

                if reached_base:
                    self.publish_feedback("reached the base station")
                    rospy.loginfo("robot-%s reached the base station", self.robot_id)
                else:
                    self.publish_feedback("failed to reach the base station")
                    rospy.loginfo("robot-%s failed to reach the base station", self.robot_id)
                    break

            else:
                _result = True

        if self._cancelled or self._preempted or not _result:
            result = rasberry_coordination.msg.CollectTrayResult(task_id=self.task.task_id, success=False)
            self.collect_tray_action.set_aborted(result)
            # Send robot to base
            self._go_to_base()
            result = self._topo_nav.get_result()
            if result is None:
                reached_base = False
                self.mode = 7
                self.goal_node = "none"
                self.start_time = rospy.get_rostime()
            else:
                reached_base = result.success
                self.mode = 0
                self.goal_node = "none"
                self.start_time = rospy.get_rostime()
        else:
            # if _result
            result = rasberry_coordination.msg.CollectTrayResult(task_id=self.task.task_id, success=True)
            self.collect_tray_action.set_succeeded(result)
            self.mode = 0
            self.goal_node = "none"
            self.start_time = rospy.get_rostime()

        self._cancelled = False
        self._preempted = False
        self.task = None

    def _preempt_cb(self, ):
        """preempt callback for collect_tray action
        """
        self._topo_nav.cancel_goal()
        self._cancelled = True
        self._preempted = True

    def publish_feedback(self, msg):
        self._fb_msg.task_id = self.task.task_id
        self._fb_msg.mode = str(self.mode)
        self._fb_msg.closest_node = self.closest_node
        self._fb_msg.route = msg
        self.collect_tray_action.publish_feedback(self._fb_msg)
        rospy.sleep(0.01)
