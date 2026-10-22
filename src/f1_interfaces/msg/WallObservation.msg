# Header stamp is the ORIGINAL LaserScan first-ray stamp, never receive time.
std_msgs/Header header
string session
uint64 sequence
bool valid
string reason
# base_link x forward, y left; left wall: (-sin(angle),cos(angle)).p=distance.
float64 distance_m
float64 tangent_rad
float64 residual_m
float64 support_m
uint32 inliers
float64 inlier_ratio
float64[2] start_xy
float64[2] end_xy
bool front_observed
float64 front_clearance_m
float64 front_valid_fraction
bool rigid_collision
# Bounded turn sweep from configured turn_sweep_rad; unknown sectors are false.
bool left_turn_clear
bool right_turn_clear
