# State estimate reference time and map frame. Integrated planning selects
# /planning/vehicle/state; bridge /vehicle/state is a separate estimator.
std_msgs/Header header

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_GPS_IMU=1
uint8 SOURCE_EGO_STATUS=2

uint8 source
bool valid

# SI: position m, yaw rad in the selected map basis, speed m/s,
# acceleration m/s^2, yaw_rate rad/s. Check producer ENU/UTM yaw configuration.
float64 x
float64 y
float64 yaw
float64 speed
float64 acceleration
float64 yaw_rate
