#include "car_signal_info.h"
#include "../other.h"

namespace i2v
{
CarSignalInfo1::CarSignalInfo1(const uint8_t* raw_data)
{
	read_size_ = 0;

	uint8_t* uint8_p = (uint8_t*)&raw_data[ADDBIT_ID / 8];//車灯器ID
	id_ = *uint8_p & BIT_ID_MASK;
	id_ >>= BIT_ID_SIHFT;
	uint8_p = (uint8_t*)&raw_data[ADDBIT_OUTPUT_CHANGE_COUNT / 8];//灯色出力変化数(K)
	uint8_t output_change_count_ = *uint8_p & BIT_OUTPUT_CHANGE_COUNT_MASK;
	read_size_ += HEADER_SIZE;//この車灯器情報のraw dataとしての全体サイズ

	for(int cou=0; cou<output_change_count_; cou++)
	{
		size_t data_start_ptr = HEADER_SIZE + CarSignalInfo2::ALL_SIZE * cou;// + 1;
		CarSignalInfo2 info2data(&raw_data[data_start_ptr]);
		info2_list_.push_back(info2data);
		read_size_ += CarSignalInfo2::ALL_SIZE;
	}
}

uint8_t CarSignalInfo1::getID() const{return id_;}
uint8_t CarSignalInfo1::getOutputChangeCount() const{return (uint8_t)info2_list_.size();}
CarSignalInfo2 CarSignalInfo1::getInfo2(const size_t index) const{return info2_list_[index];}
size_t CarSignalInfo1::getInfo2Size() const{return info2_list_.size();}
size_t CarSignalInfo1::getReadSize() const{return read_size_;}



CarSignalInfo2::CarSignalInfo2(const uint8_t* raw_data)
{
	circle_color_display_ = *((uint8_t*)&raw_data[ADDBIT_CIRCLE_COLOR_DISPLAY / 8]);//丸信号灯色表示
	arrow_color_display_ = *((uint8_t*)&raw_data[ADDBIT_ARROW_COLOR_DISPLAY / 8]);//青矢信号表示方向
	countdown_stop_flag_ = ((raw_data[ADDBIT_COUNTDOWN_STOP_FLAG / 8] & BIT_COUNTDOWN_STOP_FLAG_MASK) > 0) ? true : false;
	uint8_t uint8_2[2];
	uint8_2[0] = raw_data[ADDBIT_MIN_REMAINING_SEC_10M / 8+1];
	uint8_2[1] = raw_data[ADDBIT_MIN_REMAINING_SEC_10M / 8+0];
	uint8_2[1] &= BIT_MIN_REMAINING_SEC_10M_MASK;
	min_remaining_sec_10m_ = *((uint16_t*)uint8_2);
	max_remaining_sec_10m_ = *((uint16_t*)&raw_data[ADDBIT_MAX_REMAINING_SEC_10M / 8]);//最大残秒数(0.1秒)
	buf_rev((uint8_t*)&max_remaining_sec_10m_, 2);
}

uint8_t CarSignalInfo2::getCircleColorDisplay() const {return circle_color_display_;}
uint8_t CarSignalInfo2::getArrowColorDisplay() const {return arrow_color_display_;}
bool CarSignalInfo2::getCountdownStopFlag() const {return countdown_stop_flag_;}
uint16_t CarSignalInfo2::getMinRemainingSec10m() const {return min_remaining_sec_10m_;}
uint16_t CarSignalInfo2::getMaxRemainingSec10m() const {return max_remaining_sec_10m_;}

}