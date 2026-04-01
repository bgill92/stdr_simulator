# Laser sensor description
float32 max_angle
float32 min_angle
float32 max_range
float32 min_range
int32 num_rays

stdr_msgs/Noise noise

float32 frequency

string frame_id
geometry_msgs/Pose2D pose  # sensor pose, relative to robot center
