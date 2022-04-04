Header header
bool exist
float64 cmd_acc #指令加速度
float64 target_scheduled_achievement_time #達成予定時刻
float64 Lt #目標車間距離(自車速度から計算した車間距離)
float64 Lt_prime #目標車間距離(先行車両から計算した車間距離)
float64 L0 #現在車間距離
float64 L0_Lt #現在の速度で止まれる最短の距離
float64 current_velocity #自車両の速度
float64 vehicle_ahead_velocity #前方車両の速度
float64 current_acc #自車両の速度
float64 vehicle_ahead_acc #前方車両の加速度(mobileye param)
float64 vehicle_ahead_acc2 #前方車両の加速度(差分)
float64 relative_vel #自車両と前方車両との相対速度
float64 relative_acc #自車両と前方車両との相対加速度
float64 expected_collision_time #衝突予想時間
float64 obs_velocity_kmh
uint8 obstacle_status
