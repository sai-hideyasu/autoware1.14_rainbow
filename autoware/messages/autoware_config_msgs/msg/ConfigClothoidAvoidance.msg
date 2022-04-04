int32 AVOID_GO = 1 #回避経路実行
int32 AVOID_RETURN = 0 #回避経路終了

int32 AVOID_KAIND_DETOUR = 0 #追い抜き
int32 AVOID_KAIND_WAIT = 1 #対向車待ち

Header header
int32 avoid_flag #回避経路実行フラグ(AVOID_GO,AVOID_RETURN)
int32 avoid_kaind_flag #回避の種類
float64 move_lane_length
float64 clothoid_length
float64 phiV
float64 phiU
float64 step_count
