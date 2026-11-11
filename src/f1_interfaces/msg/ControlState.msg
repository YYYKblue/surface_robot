builtin_interfaces/Time stamp
string connection_id
string state
string reason
# Most recent lease revocation reason, retained across the neutral handshake.
string last_stop_reason
bool connected
bool motion_authorized
bool allow_fifth_thruster
string owner
# Observed CTRL wire version (0 before the first report). Compatibility does
# not imply fresh telemetry, neutral handshake completion or a motion lease.
uint8 firmware_protocol_version
bool firmware_protocol_compatible
# Observed firmware values, only current while ctrl_age_sec is within the limit.
string source
uint32 epoch
bool armed
string result
string mode
uint8 speed_percent
string direction
bool have_sequence
uint32 ack_sequence
uint32 sent_sequence
float64 ctrl_age_sec
float64 ack_age_sec
float64 command_remaining_sec
float64 imu_age_sec
uint32 imu_status
# Raw RC v2 telemetry for diagnostics only. rc_valid also requires a recent
# report; channels are not executed outputs or evidence of control ownership.
bool rc_valid
uint8 rc_flags
uint32 rc_sequence
float64 rc_age_sec
uint16[14] rc_channels
# Target PWM from CTRL, not actual ramp output, RPM or measured thrust.
uint16[4] target_pwm_bl_br_fl_fr
bool brush_enabled
int16 brush_percent
bool thrust_enabled
int16 thrust_percent
uint64 rejected_commands
uint64 invalid_lines
uint64 tx_frames
uint64 reconnects
float64 max_tx_gap_sec
