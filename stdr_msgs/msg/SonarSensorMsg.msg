# Sonar sensor description
float32 max_range
float32 min_range
float32 cone_angle

float32 frequency
stdr_msgs/Noise noise

string frame_id
geometry_msgs/Pose2D pose  # sensor pose, relative to robot center
