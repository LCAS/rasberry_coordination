#! /usr/bin/env python
# ----------------------------------
# @author: gpdas, marc-hanheide
# @email: pdasgautham@gmail.com, marc@hanheide.net
# @date:
# ----------------------------------

import operator
import Queue

import rospy

import strands_executive_msgs.msg
import strands_executive_msgs.srv
import strands_navigation_msgs.srv
import topological_navigation.route_search
import topological_navigation.tmap_utils

import rasberry_coordination.robot
import rasberry_coordination.srv


class Coordinator:
    def __init__(self, local_storage, charging_node, base_stations, picker_ids=[], robot_ids=[], unified=False):
        self.ns = "/rasberry_coordination/"

        self.picker_ids = picker_ids
#        self.robot_ids = robot_ids

        if unified:
            # create only one robot
            self.robot_ids = robot_ids[:1]
            self.robots = {robot_id:rasberry_coordination.robot.Robot(robot_id, local_storage, charging_node, base_stations[robot_id], unified) for robot_id in self.robot_ids}
        else:
            self.robot_ids = robot_ids
            self.robots = {robot_id:rasberry_coordination.robot.Robot(robot_id, local_storage, charging_node, base_stations[robot_id], unified) for robot_id in self.robot_ids}

        self.advertise_services()
        # don't queue more than 1000 tasks
        self.tasks = Queue.PriorityQueue(maxsize=1000)
        self.last_id = 0

        self.processing_tasks = {}
        self.completed_tasks = {}

        self.task_robot = {}

        # return a list of (key, value) tuples sorted based on values
        self.closest_robot = lambda x: sorted(x.items(), key=operator.itemgetter(1))

        self.tmap = None
        self.rec_map = False
        rospy.Subscriber("topological_map", strands_navigation_msgs.msg.TopologicalMap, self.map_cb)
        rospy.loginfo("Waiting for Topological map ...")

        while not self.rec_map:
            rospy.sleep(rospy.Duration.from_sec(0.1))

        self.route_search = topological_navigation.route_search.TopologicalRouteSearch(self.topo_map)

        # TODO: there are service proxies in coordinator for services in picker_state_monitor and vice versa
        # if wait_for_service, it would be blocking
        self.picker_task_updates_pub = rospy.Publisher("/picker_state_monitor/task_updates", rasberry_coordination.msg.TaskUpdates, queue_size=5)

        rospy.loginfo("coordinator initialised")

    def map_cb(self, msg):
        """This function receives the Topological Map
        """
        self.topo_map = msg
        self.rec_map = True

    def get_robot_state_ros_srv(self, req):
        """get the state of a robot"""
        resp = rasberry_coordination.srv.RobotStateResponse()
        if req.robot_id in self.robot_ids:
            resp.state, resp.goal_node, resp.start_time = self.robots[req.robot_id].get_state()
        else:
            err_msg = "%s is not a among the robots configured" %(req.robot_id)
            rospy.logerr(err_msg)
        return resp

    get_robot_state_ros_srv.type = rasberry_coordination.srv.RobotState

    def get_robot_states_ros_srv(self, req):
        """get the state of a set of robots"""
        resp = rasberry_coordination.srv.RobotStatesResponse()
        for robot_id in req.robot_ids:
            if robot_id in self.robot_ids:
                state, goal_node, start_time = self.robots[robot_id].get_state()
                resp.states.append(state)
                resp.goal_nodes.append(goal_node)
                resp.start_times.append(start_time)
            else:
                resp.states.append("")
                resp.goal_nodes.append("")
                resp.start_times.append(rospy.Time())
                err_msg = "%s is not a among the robots configured" %(robot_id)
                rospy.logerr(err_msg)
        return resp

    get_robot_states_ros_srv.type = rasberry_coordination.srv.RobotStates

    def add_task_ros_srv(self, req):
        """
        Adds a task into the task execution framework.
        """
        self.last_id += 1
        task_id = self.last_id
        req.task.task_id = task_id
        rospy.loginfo('received task: %s' % req.task)
        self.tasks.put(
            (task_id, req.task)
        )
        return task_id

    add_task_ros_srv.type = strands_executive_msgs.srv.AddTask

    def cancel_task_ros_srv(self, req):
        success = True
        # TODO: coordinator cancelling task after picker request
        rospy.loginfo("TODO: coordinator cancelling task after picker request")
#        self.robots[req.robot_id].collect_tray.cancel_all_goals()
        return success

    cancel_task_ros_srv.type = rasberry_coordination.srv.CancelTask

    def advertise_services(self):
        """
        Adverstise ROS services.
        Only call at the end of constructor to avoid calls during construction.
        """
        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service = getattr(self, attr)
                rospy.Service(
                    self.ns+attr[:-8],
                    service.type,
                    service
                )
                rospy.loginfo('advertised %s' % attr[:-8])

    def handle_task(self):
        """
        this is to be overwritten if used as a superclass
        """
        pass

    def get_idle_robots(self, ):
        """
        """
        idle_robots = []
        for robot_id in self.robot_ids:
            if self.robots[robot_id].is_idle() and self.robots[robot_id].closest_node != "none":
                # if the robot is idle and its closest_node is not 'none' (topo_nav ready for that robot)
                idle_robots.append(robot_id)
            else:
#                rospy.loginfo(robot_id)
#                rospy.loginfo(self.robots[robot_id].closest_node)
#                rospy.loginfo(self.robots[robot_id].is_idle())
                pass
        return idle_robots

    def find_closest_robot(self, task, idle_robots):
        """find the robot closest to the task location (picker_node)

        Keyword arguments:

        task - strands_executive_msgs.Task
        idle_robots - list of robot_ids
        """
        goal_node = task.start_node_id
        robot_dists = {}
        for robot_id in idle_robots:
            start_node = self.robots[robot_id].closest_node
            if start_node =="none" or goal_node == "none" or start_node is None or goal_node is None:
                route_dists = [float("inf")]
            elif start_node != goal_node:
                route_nodes, route_edges, route_dists = self.get_path_details(start_node, goal_node)
            else:
                route_dists = [0]

            if abs(sum(route_dists)) != float("inf"):
                robot_dists[robot_id] = sum(route_dists)

        return self.closest_robot(robot_dists)[0][0]

    def get_path_details(self, start_node, goal_node):
        """get route_nodes, route_edges and route_distance from start_node to goal_node

        Keyword arguments:
        start_node -- name of the starting node
        goal_node -- name of the goal node
        """
        route_distance = []
        route = self.route_search.search_route(start_node, goal_node)
        if route is None:
            rosinfomsg = "no route between %s and %s" %(start_node, goal_node)
            rospy.loginfo(rosinfomsg)
            return ([], [], [float("inf")])
        route_nodes = route.source
        route_edges = route.edge_id

        edge_to_goal = self.get_edges_between_nodes(route_nodes[-1], goal_node)
        route_edges.append(edge_to_goal[0])
        route_nodes.append(goal_node)

        for i in range(len(route_nodes) - 1):
            route_distance.append(self.get_distance_between_nodes(route_nodes[i], route_nodes[i + 1]))

        return (route_nodes, route_edges, route_distance)

    def get_node(self, node):
        """get_node: Given a node name return its node object.
        A wrapper for the get_node function in tmap_utils

        Keyword arguments:

        node -- name of the node in topological map"""
        return topological_navigation.tmap_utils.get_node(self.topo_map, node)

    def get_distance_between_nodes(self, from_node, to_node):
        """get_distance_between_nodes: Given names of two nodes, return the distance of the edge
        between their node objects. A wrapper for the get_distance_to_node function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        from_node_obj = self.get_node(from_node)
        to_node_obj = self.get_node(to_node)
        return topological_navigation.tmap_utils.get_distance_to_node(from_node_obj, to_node_obj)

    def get_edges_between_nodes(self, from_node, to_node):
        """get_edges_between_nodes: Given names of two nodes, return the direct edges
        between their node objects. A wrapper for the get_edges_between function in tmap_utils.
        Works only for adjacent nodes.

        Keyword arguments:

        from_node -- name of the starting node
        to_node -- name of the ending node name"""
        edge_ids = []
        edges = topological_navigation.tmap_utils.get_edges_between(self.topo_map, from_node, to_node)
        for edge in edges:
            edge_ids.append(edge.edge_id)
        return edge_ids

    def run(self):
        # get idle robots
        idle_robots = self.get_idle_robots()
#        rospy.loginfo(idle_robots)

        while not rospy.is_shutdown():
#            rospy.loginfo(idle_robots)
            if len(idle_robots) != 0:
                if not self.tasks.empty():
                    msg = "unassigned tasks present. no. idle robots: %d" %(len(idle_robots))
                    rospy.loginfo(msg)

                # try assigning a robot if there is an idle robot
                try:
                    (task_id, task) = self.tasks.get(True, 1)
                    self.processing_tasks[task_id] = task
                    rospy.loginfo('process task %d' % task_id)

                    # find the closest robot
                    robot_id = self.find_closest_robot(task, idle_robots)
                    rosinfomsg = "selected robot-%s to task %d" %(robot_id, task.task_id)
                    rospy.loginfo(rosinfomsg)

                    self.task_robot[task_id] = robot_id

                    # call collecttray action -- action selection must be from the task details
                    collect_tray_goal = rasberry_coordination.msg.CollectTrayGoal()

                    collect_tray_goal.task_id = task_id
                    collect_tray_goal.picker_node = task.start_node_id
#                    collect_tray_goal.storage_node = "none" # uses local_storage
                    # hard coding duration now
                    collect_tray_goal.min_load_duration.secs = 10.
                    collect_tray_goal.max_load_duration.secs = 20.
                    collect_tray_goal.min_unload_duration.secs = 10.
                    collect_tray_goal.max_unload_duration.secs = 20.
                    self.robots[robot_id].collect_tray.send_goal(collect_tray_goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)

                    task_state = rasberry_coordination.msg.TaskUpdates()
                    task_state.task_id = task_id
                    task_state.robot_id = self.task_robot[task_id]
                    task_state.state = "ACCEPT"
                    for i in range(3):
                        self.picker_task_updates_pub.publish(task_state)
                        rospy.sleep(0.01)

                except Queue.Empty:
                    # update idle robots
                    idle_robots = self.get_idle_robots()
                    continue

            # update idle robots
            idle_robots = self.get_idle_robots()

    def done_cb(self, status, result):
        """done callback for collect_tray action
        """
        # TODO:
        if result.success:
            self.task_robot.pop(result.task_id)
            task = self.processing_tasks.pop(result.task_id)
            self.completed_tasks[result.task_id] = task

        else:
            pass

    def feedback_cb(self, fb):
        """feedback callback for collect_tray action
        """
        if fb.route == "reached picker":
            task_state = rasberry_coordination.msg.TaskUpdates()
            task_state.task_id = fb.task_id
            task_state.robot_id = self.task_robot[fb.task_id]
            task_state.state = "ARRIVED"
            for i in range(3):
                self.picker_task_updates_pub.publish(task_state)
                rospy.sleep(0.01)

        elif fb.route == "tray loaded":
            # in case this is set by robot after a delay
            task_state = rasberry_coordination.msg.TaskUpdates()
            task_state.task_id = fb.task_id
            task_state.robot_id = self.task_robot[fb.task_id]
            task_state.state = "LOADED"
            for i in range(3):
                self.picker_task_updates_pub.publish(task_state)
                rospy.sleep(0.01)

        elif fb.route == "reached storage":
            # TODO: tray unloading is not considered
            # no need to do anything here at the moment
            pass

        elif fb.route == "tray unloaded":
            task_state = rasberry_coordination.msg.TaskUpdates()
            task_state.task_id = fb.task_id
            task_state.robot_id = self.task_robot[fb.task_id]
            task_state.state = "DELIVERED"
            for i in range(3):
                self.picker_task_updates_pub.publish(task_state)
                rospy.sleep(0.01)

        elif ((fb.route == "failed to reach the picker") or
              (fb.route == "failed to reach storage") or
              (fb.route == "failed to load tray") or
              (fb.route == "failed to unload tray")):
            # TODO: fb saying failed. nothing to do at this stage
            pass

    def on_shutdown(self, ):
        print "shutting down all actions"
        pass
        for robot_id in self.robot_ids:
            print robot_id, self.robots[robot_id].is_idle()
            if not self.robots[robot_id].is_idle():
                self.robots[robot_id]._topo_nav.cancel_all_goals()

if __name__ == '__main__':
    rospy.init_node('simple_task_coordinator', anonymous=True)
    c = Coordinator()
    c.run()
    rospy.spin()
