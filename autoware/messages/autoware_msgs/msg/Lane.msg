uint8 SIGNAL_SELECT_RED = 0
uint8 SIGNAL_SELECT_GREEN = 1
uint8 SIGNAL_SELECT_UNKNOWN = 2

Header header
int32 increment
int32 lane_id
Waypoint[] waypoints

uint32 lane_index
float32 cost
float32 closest_object_distance
float32 closest_object_velocity
bool is_blocked

#scw add
uint8 signal_select
float64 pitch_average
