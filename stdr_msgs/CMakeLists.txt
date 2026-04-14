cmake_minimum_required(VERSION 3.16)
project(stdr_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/CO2SensorMeasurementMsg.msg"
  "msg/CO2SensorMsg.msg"
  "msg/CO2Source.msg"
  "msg/CO2SourceVector.msg"
  "msg/FootprintMsg.msg"
  "msg/KinematicMsg.msg"
  "msg/LaserSensorMsg.msg"
  "msg/Noise.msg"
  "msg/RfidSensorMeasurementMsg.msg"
  "msg/RfidSensorMsg.msg"
  "msg/RfidTag.msg"
  "msg/RfidTagVector.msg"
  "msg/RobotIndexedMsg.msg"
  "msg/RobotIndexedVectorMsg.msg"
  "msg/RobotMsg.msg"
  "msg/SonarSensorMsg.msg"
  "msg/SoundSensorMeasurementMsg.msg"
  "msg/SoundSensorMsg.msg"
  "msg/SoundSource.msg"
  "msg/SoundSourceVector.msg"
  "msg/ThermalSensorMeasurementMsg.msg"
  "msg/ThermalSensorMsg.msg"
  "msg/ThermalSource.msg"
  "msg/ThermalSourceVector.msg"
  "srv/AddCO2Source.srv"
  "srv/AddRfidTag.srv"
  "srv/AddSoundSource.srv"
  "srv/AddThermalSource.srv"
  "srv/DeleteCO2Source.srv"
  "srv/DeleteRfidTag.srv"
  "srv/DeleteSoundSource.srv"
  "srv/DeleteThermalSource.srv"
  "srv/LoadExternalMap.srv"
  "srv/LoadMap.srv"
  "srv/MoveRobot.srv"
  "srv/RegisterGui.srv"
  "action/DeleteRobot.action"
  "action/RegisterRobot.action"
  "action/SpawnRobot.action"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  action_msgs)

ament_package()
