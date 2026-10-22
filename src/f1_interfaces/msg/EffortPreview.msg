std_msgs/Header header
builtin_interfaces/Time observation_stamp
builtin_interfaces/Time valid_until
builtin_interfaces/Time direction_until
string phase
string reason
bool valid
bool hardware_output_enabled
# Same exclusive direction enum as ActuatorCommand (0..4); PWM gear only.
uint8 direction
uint8 speed_percent
bool brush_enabled
float64 brush_percent
