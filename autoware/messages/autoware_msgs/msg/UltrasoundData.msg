uint32 POS_FRONT_LEFT = 1
uint32 POS_FRONT_CENTER = 2
uint32 POS_FRONT_RIGHT = 4
uint32 POS_BACK_LEFT = 8
uint32 POS_BACK_CENTER = 16
uint32 POS_BACK_RIGHT = 32
uint32 POS_LEFT_FRONT = 64
uint32 POS_LEFT_SHALLOWFRONT = 128
uint32 POS_LEFT_SHALLOWBACK = 256
uint32 POS_LEFT_BACK = 512
uint32 POS_RIGHT_FRONT = 1024
uint32 POS_RIGHT_SHALLOWFRONT = 2048
uint32 POS_RIGHT_SHALLOWBACK = 4096
uint32 POS_RIGHT_BACK = 8192

Header header
uint32 pin_id #データ取得に使用したarduino pin
float64 conversion_time #前回のビット転換時間と新しいビット転換時間から出した時間　周波数はこれの逆数で取得
int32 bit_interval #ONであるビットの間隔　−１はなし　０ならONビットが１つのみ
float64 object_distance #障害物までの距離
uint32 position_flag #車両のどの位置に属するかのフラグ
uint8[] bit_history #ビットの履歴
