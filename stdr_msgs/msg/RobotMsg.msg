geometry_msgs/Pose2D initial_pose

stdr_msgs/FootprintMsg footprint

# Point about which the robot rotates, in robot body frame.
# Defaults to (0, 0) — the robot's geometric centre.
geometry_msgs/Point center_of_rotation

stdr_msgs/LaserSensorMsg[] laser_sensors
stdr_msgs/SonarSensorMsg[] sonar_sensors
stdr_msgs/RfidSensorMsg[] rfid_sensors
stdr_msgs/CO2SensorMsg[] co2_sensors
stdr_msgs/SoundSensorMsg[] sound_sensors
stdr_msgs/ThermalSensorMsg[] thermal_sensors

stdr_msgs/KinematicMsg kinematic_model
