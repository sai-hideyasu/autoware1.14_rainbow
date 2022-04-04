Header header
float64 judg_obj_pose_y #正面判定の車両相対Y座標(横方向)
float64 obs_delete_time #正面オブジェクトの更新がなかった場合にデーターから消去する秒数
float64 min_free_running_distance #最小車間距離
#float64 target_deceleration #目標減速度
float64 control_cycle #制御周期 車間距離の計算での処理遅延時間を決める際に使用する delay_time = 1 / control_cycle * 2
float64 expected_collision_time_th #衝突予想時間のしきい値
float64 avoid_collision_corr1
float64 avoid_collision_corr2
