uint8 UNKNOWN = 0
uint8 TEMPORARY_STOPPER = 1
uint8 SIGNAL = 2
uint8 OBSTACLE = 3
uint8 ONCOMING = 4

Header header
float64 distance #停止線までの距離
uint8 send_process #停止線の種類　上の定数を意味する
float64 fixed_velocity #停止線に設定されている速度
float64 reach_time #停止線までの到達時間
float64 distance_rss
