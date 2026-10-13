# Time-continuous target estimate produced from detections and IMU yaw rate.

uint8 LOST=0
uint8 TRACKING=1
uint8 PREDICTING=2

std_msgs/Header header
bool target_valid
uint8 tracking_state
float32 bearing_rad
float32 bearing_rate_rad_s
float32 area_ratio
float32 confidence
float32 detection_age_sec
bool imu_valid
float32 yaw_rate_rad_s
