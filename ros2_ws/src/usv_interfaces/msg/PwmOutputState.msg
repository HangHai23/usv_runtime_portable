# PWM actuator command after validation, limiting and conversion.

std_msgs/Header header
float32[2] requested_percent
float32[2] applied_percent
uint16[2] pulse_width_us
uint16[2] controller_angle_deg
bool watchdog_active
bool emergency_stop_active
bool dry_run
string source
