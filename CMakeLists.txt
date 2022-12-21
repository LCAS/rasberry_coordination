cmake_minimum_required(VERSION 2.8.3)
project(rasberry_coordination)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  diagnostic_msgs
  message_generation
  rasberry_des
  rasberry_people_perception
  rosbridge_server
  rosduct
  rosduct
  rospy
  std_msgs
  std_srvs
  strands_executive_msgs
  strands_navigation_msgs
  topic_tools
  topological_navigation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AgentRegistrationList.msg
  AgentRegistration.msg
  AgentStateList.msg
  AgentState.msg
  AgentLocationList.msg
  AgentLocation.msg
  FullLocation.msg
  Interruption.msg
  MarkerDetails.msg
  Module.msg
  NewAgentConfig.msg
  ServiceQuery.msg
  ServiceResponse.msg
  TaskDetails.msg
  TasksDetails.msg
  TopoLocation.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddNodeTask.srv
  StringList.srv
  String.srv
)

generate_messages(
  DEPENDENCIES std_msgs strands_executive_msgs diagnostic_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/abstract_task_executor_node.py
  scripts/add_agent.py
  scripts/initialise_debug_agent_position.py
  scripts/rviz_markers.py
  scripts/ui_speaker_broadcast.py
  scripts/ui_speaker.py
  scripts/visualise_agent_markers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach (dir launch config scripts msg srv)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}/)
endforeach(dir)
