# HPR receipt time on the ROS clock. heading uses the antenna frame and
# ANT1 -> ANT2 baseline; bow_heading uses the base frame and its forward axis.
std_msgs/Header header
# Receiver UTC time of day as hhmmss.ss; empty when unavailable.
string utc
# Degrees clockwise from true north. NaN when valid is false.
float64 heading_deg
# HPR solution quality; -1 if absent from the sentence or the measurement expired.
int8 quality
bool valid
