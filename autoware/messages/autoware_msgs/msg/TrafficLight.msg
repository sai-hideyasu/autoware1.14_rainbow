uint8 DEVICE_NO = 0
uint8 DEVICE_YOLO = 1
uint8 DEVICE_I2V = 2

Header header
int32 traffic_light #信号機の色
uint8 device_type #色情報を送信したデバイス
int32 i2v_orig_light #i2vの現在の出力値
uint16 i2v_change_time
