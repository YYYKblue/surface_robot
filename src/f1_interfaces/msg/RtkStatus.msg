std_msgs/Header header
bool connected
# Latest GGA quality in this serial session, -1 when unavailable.
int8 fix_quality
# Legacy heading fields always report -1, false and -1 seconds.
int8 heading_quality
bool fix_valid
bool heading_valid
# Monotonic seconds since the last valid observation, -1 if never seen.
float64 fix_age_sec
float64 heading_age_sec
# Values from the latest parsed GGA, -1 or NaN when unavailable.
string fix_utc
int16 satellites
float64 hdop
# Serial/open error while disconnected, empty when connected.
string serial_error
# Total malformed GGA sentences or oversized lines since driver startup.
uint64 rejected_lines
