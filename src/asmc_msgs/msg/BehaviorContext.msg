std_msgs/Header header

uint8 KEEP=0
uint8 FOLLOW=1
uint8 LEFT_CHANGE=2
uint8 RIGHT_CHANGE=3
uint8 STOP=4
uint8 YIELD=5
uint8 EMERGENCY_STOP=6
uint8 AVOID=7
uint8 MERGE=8

uint32 command_id
bool valid
uint8 selected_behavior
bool force_stop

bool has_desired_speed
float64 desired_speed
bool has_max_speed
float64 max_speed

bool has_target_lane
int8 target_lane_offset
bool has_target_d
float64 target_d

bool has_stop_target
float64 stop_x
float64 stop_y

bool has_lead_vehicle
int32 lead_vehicle_id

string target_link_id
