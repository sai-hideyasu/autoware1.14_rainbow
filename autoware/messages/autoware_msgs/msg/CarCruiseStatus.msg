int32 TRACKING_NO = 0
int32 TRACKING_DECELERATION1 = 1
int32 TRACKING_DECELERATION2 = 2
int32 TRACKING_EMG_DECELERATION = 3
int32 TRACKING_STOP = 4
int32 TRACKING_ACCELERATION = 5

Header header
float64 relative_velocity_mps #前方車両の相対速度
float64 distance_x_m #前方車両までの距離
float64 distance_rss_m #責任車間距離
float64 expected_collision_time #衝突予想時間
int32 tracking_mode #前方車両追跡モード
bool accel_release #アクセルリリースモードにするか？
