# header.stamp is the sender's position measurement time, not receive time.
# Cartesian global coordinates in metres; frame_id identifies the shared frame.
std_msgs/Header header
builtin_interfaces/Time received_stamp
float64 x
float64 y
float64 z
string sender
