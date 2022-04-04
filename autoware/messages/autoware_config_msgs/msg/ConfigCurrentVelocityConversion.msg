int32 VELOCITY_MODE_CONSTANT = 0
int32 VELOCITY_MODE_CONSTANT_DIRECT = 1
int32 VELOCITY_MODE_CAN = 2
int32 VELOCITY_MODE_CAN_DIRECT = 3
int32 VELOCITY_MODE_MOBILEYE_CONSTANT = 4
int32 VELOCITY_MODE_MOBILEYE_CAN = 5

Header header
bool enable
int32 velocity_mode
float64 search_distance
float64 search_step
uint8 search_step_count
float32 constant_velocity
float32 velocity_max
