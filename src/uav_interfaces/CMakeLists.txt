cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DebrisTrack.msg"
  "msg/DebrisTrackArray.msg"
  "msg/PredictedState.msg"
  "msg/DebrisPrediction.msg"
  "msg/DebrisPredictionArray.msg"
  "msg/DebrisRisk.msg"
  "msg/DebrisRiskArray.msg"
  "msg/AvoidanceCommand.msg"
  "msg/FlightStatus.msg"
  "msg/SurvivorDetection.msg"
  "msg/SurvivorDetectionArray.msg"
  "msg/SurvivorLocation.msg"
  "msg/SurvivorLocationArray.msg"
  "msg/SearchGoal.msg"
  "msg/MissionCommand.msg"
  "msg/MissionStatus.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
