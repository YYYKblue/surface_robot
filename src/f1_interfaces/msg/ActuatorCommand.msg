# Obtain a fresh token from acquire_control after every stop, timeout or takeover.
# Times use the host system clock. Preserve the original computation time/deadline.
string token
uint64 sequence
builtin_interfaces/Time computed_at
builtin_interfaces/Time valid_until
# Optional absolute pulse end. Zero uses valid_until. Ends motion and auxiliaries
# while a fresh STOP command can keep the lease; resends never move this time.
builtin_interfaces/Time direction_until
# Wire v3: exactly one chassis direction; firmware performs wheel mixing.
uint8 STOP=0
uint8 FORWARD=1
uint8 REVERSE=2
uint8 LEFT=3
uint8 RIGHT=4
uint8 direction
# Main motor PWM output: integer 0..100, not measured speed or thrust.
# Zero stops only the main motors; auxiliary outputs remain independent.
# v3 25% = 100 us PWM offset; the old 25 gear was 125 us.
uint8 speed_percent 25
uint8 FRONT=0
uint8 BOTH=1
uint8 BACK=2
uint8 mode
bool brush_enabled
float32 brush_percent
bool thrust_enabled
float32 thrust_percent
