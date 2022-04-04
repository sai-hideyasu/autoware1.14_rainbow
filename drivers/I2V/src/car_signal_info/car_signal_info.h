#ifndef CAR_SIGNAL_INFO
#define CAR_SIGNAL_INFO

#include <iostream>
#include <vector>

namespace i2v
{

class CarSignalInfo2
{
public:
	static const size_t BIT_CIRCLE_COLOR_DISPLAY = 8;
	static const size_t BIT_ARROW_COLOR_DISPLAY = 8;
	static const size_t BIT_COUNTDOWN_STOP_FLAG = 1;
	static const uint8_t BIT_COUNTDOWN_STOP_FLAG_MASK = 0x80;
	static const size_t BIT_MIN_REMAINING_SEC_10M = 15;
	static const uint8_t BIT_MIN_REMAINING_SEC_10M_MASK = 0x7F;
	static const size_t BIT_MAX_REMAINING_SEC_10M = 16;

	static const size_t ADDBIT_CIRCLE_COLOR_DISPLAY = 0;
	static const size_t ADDBIT_ARROW_COLOR_DISPLAY = ADDBIT_CIRCLE_COLOR_DISPLAY + BIT_CIRCLE_COLOR_DISPLAY;
	static const size_t ADDBIT_COUNTDOWN_STOP_FLAG = ADDBIT_ARROW_COLOR_DISPLAY + BIT_ARROW_COLOR_DISPLAY;
	static const size_t ADDBIT_MIN_REMAINING_SEC_10M = ADDBIT_COUNTDOWN_STOP_FLAG + BIT_COUNTDOWN_STOP_FLAG;
	static const size_t ADDBIT_MAX_REMAINING_SEC_10M = ADDBIT_MIN_REMAINING_SEC_10M + BIT_MIN_REMAINING_SEC_10M;
	static const size_t ALL_SIZE = (ADDBIT_MAX_REMAINING_SEC_10M + BIT_MAX_REMAINING_SEC_10M) /8;
private:
	uint8_t circle_color_display_;//丸信号灯色表示
	uint8_t arrow_color_display_;//青矢信号表示方向
	bool countdown_stop_flag_;//カウントダウン停止フラグ
	uint16_t min_remaining_sec_10m_;//最小残秒数(0.1秒)
	uint16_t max_remaining_sec_10m_;//最大残秒数(0.1秒)
public:
	CarSignalInfo2(const uint8_t* raw_data);

	uint8_t getCircleColorDisplay() const;
	uint8_t getArrowColorDisplay() const;
	bool getCountdownStopFlag() const;
	uint16_t getMinRemainingSec10m() const;
	uint16_t getMaxRemainingSec10m() const;
};

class CarSignalInfo1
{
public:
	static const size_t BIT_ID = 4;
	static const uint8_t BIT_ID_MASK = 0xF0;
	static const int BIT_ID_SIHFT = 4;
	static const size_t BIT_OUTPUT_CHANGE_COUNT = 4;
	static const uint8_t BIT_OUTPUT_CHANGE_COUNT_MASK = 0x0F;

	static const size_t ADDBIT_ID = 0;
	static const size_t ADDBIT_OUTPUT_CHANGE_COUNT = ADDBIT_ID + BIT_ID;
	static const size_t HEADER_SIZE = (ADDBIT_OUTPUT_CHANGE_COUNT + BIT_OUTPUT_CHANGE_COUNT) / 8;
private:
	uint8_t id_;//車灯器ID
	std::vector<CarSignalInfo2> info2_list_;//この車灯器IDにおける車両灯器情報　灯色出力変化数(K)
	size_t read_size_;//この車灯器情報のraw dataとしての全体サイズ
public:
	CarSignalInfo1(const uint8_t* raw_data);

	uint8_t getID() const;
	uint8_t getOutputChangeCount() const;
	CarSignalInfo2 getInfo2(const size_t index) const;
	size_t getInfo2Size() const;
	size_t getReadSize() const;
};

}

#endif