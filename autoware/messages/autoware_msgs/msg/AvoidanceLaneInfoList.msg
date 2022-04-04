int8 STATUS_NORMAL = 0
int8 STATUS_AVOIDANCE_GO = 1
int8 STATUS_AVOIDANCE_RETURN = 2

Header header
#移動させるwaypointの最大幅
int8 status
float64 move_width
AvoidanceLaneInfo[] list
