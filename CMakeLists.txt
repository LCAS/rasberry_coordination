cmake_minimum_required(VERSION 3.5)
project(rasberry_coordination)

find_package(ament_cmake REQUIRED)
find_package(actionlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(message_generation REQUIRED)
find_package(rasberry_des REQUIRED)
find_package(rasberry_people_perception REQUIRED)
find_package(rosbridge_server REQUIRED)
find_package(rosduct REQUIRED)
find_package(rosduct REQUIRED)
find_package(rospy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(strands_executive_msgs REQUIRED)
find_package(strands_navigation_msgs REQUIRED)
find_package(topic_tools REQUIRED)
find_package(topological_navigation REQUIRED)


catkin_python_setup()

## Generate messages in the 'msg' folder
set(msg_files
  "msg/AgentList.msg"
  "msg/Agent.msg"
  "msg/AgentLocation.msg"
  "msg/AgentRegistration.msg"
  "msg/AgentState.msg"
  "msg/AgentRendering.msg"
  "msg/AgentHealth.msg"
  "msg/FullLocation.msg"
  "msg/Interruption.msg"
  "msg/MarkerDetails.msg"
  "msg/Module.msg"
  "msg/NewAgentConfig.msg"
  "msg/ServiceQuery.msg"
  "msg/ServiceResponse.msg"
  "msg/TaskDetails.msg"
  "msg/TasksDetails.msg"
  "msg/TopoLocation.msg"

# Scheduling
  "msg/AllSchedulableTasks.msg"
  "msg/SchedulableTasks.msg"
  "msg/Task.msg"
  "msg/TaskIdentifier.msg"
  "msg/TaskContents.msg"
  "msg/TaskSchedule.msg"
  "msg/TaskStatus.msg"
)

## Generate services in the 'srv' folder
set(srv_files
  "srv/AddNodeTask.srv"
  "srv/StringList.srv"
  "srv/String.srv"
)

# Generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces
)

# Declare dependencies
ament_export_dependencies(rosidl_default_runtime)

# Allow this package to be specified as a dependency
ament_package()


# Install mapping rules for paramater name matches within the message
install(
  FILES rosbridge_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})


#^ based on https://www.allisonthackston.com/articles/bridging-ros-ros2.html




include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#  scripts/abstract_task_executor_node.py
#  scripts/add_agent.py
#  scripts/initialise_debug_agent_position.py
#  scripts/rviz_markers.py
#  scripts/ui_speaker_broadcast.py
#  scripts/ui_speaker.py
#  scripts/visualise_agent_markers.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

foreach (dir launch config scripts msg srv)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}/)
endforeach(dir)
