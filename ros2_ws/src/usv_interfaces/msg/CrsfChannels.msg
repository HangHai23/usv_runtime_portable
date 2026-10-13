# Parsed CRSF RC_CHANNELS_PACKED (frame type 0x16).
# No vehicle-control meaning is assigned to any channel in this message.

std_msgs/Header header
uint16[16] raw
uint16[16] microseconds
float32 frame_rate_hz
uint32 valid_frame_count
uint32 crc_error_count
bool connected
