# Legacy type for recorded HPR observations.
std_msgs/Header header
# Receiver UTC time of day as hhmmss.ss; empty when unavailable.
string utc
# Degrees clockwise from true north. NaN when valid is false.
float64 heading_deg
# HPR solution quality; -1 if absent from the sentence or the measurement expired.
int8 quality
bool valid
