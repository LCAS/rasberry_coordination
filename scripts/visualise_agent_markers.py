#!/usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import rasberry_coordination.base_frame_publisher
import rasberry_coordination.markers
import sys
import rasberry_des.config_utils
import geometry_msgs.msg


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise Exception("usage: robot_markers.py <config_file>")
    else:
        config_file = sys.argv[1]

    config_data = rasberry_des.config_utils.get_config_data(config_file)
    config_keys = rasberry_des.config_utils.get_config_keys(config_file)

    # check for required parameters
    req_params = ["robot_ids", "picker_ids"]

    for key in config_keys:
        if key in req_params:
            req_params.remove(key)

    if len(req_params) != 0:
        raise Exception("not all required keys are set in the config file")
    elif config_data["robot_ids"].__class__ != list:
        raise Exception("robot_ids should be a list in the config file")
    elif len(config_data["robot_ids"]) == 0:
        raise Exception("robot_ids should not be an empty list in the config file")

    rospy.init_node("visualise_agent_markers")

    robot_ids = config_data["robot_ids"]
    n_robots = len(robot_ids)

    picker_ids = config_data["picker_ids"]
    n_pickers = len(picker_ids)

    virtual_picker_ids = []
    if "virtual_picker_ids" in config_data:
        virtual_picker_ids = config_data["virtual_picker_ids"]
    n_virtual_pickers = len(virtual_picker_ids)

    thorvald_marker_publishers = []
    picker_marker_publishers = []
    virtual_picker_marker_publishers = []
    base_frame_publishers = []
    for robot_id in robot_ids:
        thorvald_marker_publishers.append(rasberry_coordination.markers.ThorvaldMarkerPublisher(robot_id))
        base_frame_publishers.append(rasberry_coordination.base_frame_publisher.PoseBaseFramePublisher(robot_id, "/".join(("", robot_id, "robot_pose"))))

    for picker_id in picker_ids:
        picker_marker_publishers.append(rasberry_coordination.markers.HumanMarkerPublisher(picker_id))
        base_frame_publishers.append(rasberry_coordination.base_frame_publisher.PoseStampedBaseFramePublisher(picker_id, "/".join(("", picker_id, "posestamped"))))

    for virtual_picker_id in virtual_picker_ids:
        virtual_picker_marker_publishers.append(rasberry_coordination.markers.HumanMarkerPublisher(virtual_picker_id))
        base_frame_publishers.append(rasberry_coordination.base_frame_publisher.PoseStampedBaseFramePublisher(virtual_picker_id, "/".join(("", virtual_picker_id, "posestamped"))))

    while not rospy.is_shutdown():
        for i in range(n_robots):
            thorvald_marker_publishers[i].publish()
        for i in range(n_pickers):
            picker_marker_publishers[i].publish()
        for i in range(n_virtual_pickers):
            virtual_picker_marker_publishers[i].publish()
        rospy.sleep(1.0)

