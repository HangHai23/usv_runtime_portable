# Highest-confidence target detection from the vision inference node.
# Pixel coordinates use the source image coordinate system. Normalized values
# remain in 0..1 and are independent of camera resolution.

std_msgs/Header header
bool detected
int32 class_id
string class_name
float32 confidence
uint32 image_width
uint32 image_height
float32 x_min
float32 y_min
float32 x_max
float32 y_max
float32 center_x
float32 center_y
float32 width
float32 height
float32 area_ratio
float32 inference_ms
