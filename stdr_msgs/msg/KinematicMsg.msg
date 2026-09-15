# Container for the kinematic model parameters.
# Parameters are in the form a_C_D, where C is affected by D.
# ux is the linear speed
# uy is the lateral speed (for omni vehicles)
# w is the angular speed
# g is a coefficient that directly affects the angular speed

string type
# Selects how the simulated odometry belief pose relates to the true pose:
# "perfect" (alphas below ignored, odom == truth) or "velocity" (alphas
# below sampled into truth once per tick; odom is the noise-free
# integration of the same command). Empty is treated as "perfect".
string odometry_model
float32 a_ux_ux
float32 a_ux_uy
float32 a_ux_w
float32 a_uy_ux
float32 a_uy_uy
float32 a_uy_w
float32 a_w_ux
float32 a_w_uy
float32 a_w_w
float32 a_g_ux
float32 a_g_uy
float32 a_g_w
