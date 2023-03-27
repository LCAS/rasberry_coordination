cmake_minimum_required(VERSION 3.5)
project(rasberry_coordination_msgs)


############################
#
#   DEPENDENCY DEFINITION
#

# Define build tool dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define dependencies for messages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
set(msg_deps
  std_msgs
  geometry_msgs
  diagnostic_msgs
)


# Define dependencies for services
find_package(std_srvs REQUIRED)
set(srv_deps
  std_srvs
)




########################
#
#   FILE DECLARATION
#

# Declare the custom message files
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

# Declare the custom service files
set(srv_files
  "srv/AddNodeTask.srv"
  "srv/StringList.srv"
  "srv/String.srv"
)



#######################
#
#   FILE GENERATION
#

# Generate the files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES ${msg_deps}
)


###################################
#
#   DOWNSTREAM DEPENDENCY EXPORT
#

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)
ament_package()











