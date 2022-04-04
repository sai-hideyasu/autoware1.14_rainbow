#include "service_load_signal_info.h"
#include "../other.h"

namespace i2v
{

ServiceLoadSignalInfo::ServiceLoadSignalInfo(const uint8_t* raw_data, const uint8_t connection_load_count, const uint16_t service_ptr, const uint16_t reader_head_size)
{
	read_raw_data_size_ = 0;
	id_ = *((uint8_t*)&raw_data[service_ptr + ADDBIT_ID / 8]);//方路ID
	uint8_t* uint8_p = (uint8_t*)&raw_data[service_ptr + ADDBIT_TRAFFIC_DIRECTION_INFO_FLAG / 8];
	traffic_direction_info_flag_ = ((*uint8_p & BIT_TRAFFIC_DIRECTION_INFO_FLAG_MASK) > 0) ? true : false;//信号通行方向情報有無フラグ
	traffic_direction_info_ = *((uint8_t*)&raw_data[service_ptr + ADDBIT_TRAFFIC_DIRECTION_INFO / 8]);//信号通行方向情報
	read_raw_data_size_ += HEADER_SIZE;

	//車灯器情報を纏める
	for(uint8_t cou=0; cou<connection_load_count; cou++)
	{
		//uint16_t index = HEADER_SIZE + cou * SIZE_CAR_SIGNAL_INFO_POINTER;
		uint16_t car_ptr1 = *((uint16_t*)&raw_data[service_ptr + read_raw_data_size_]);//index]);
		buf_rev((uint8_t*)&car_ptr1, 2);
		read_raw_data_size_ += SIZE_CAR_SIGNAL_INFO_POINTER;
		if(car_ptr1 == 0xFFFF) continue;
		uint16_t car_ptr2 = car_ptr1 + reader_head_size;
//std::cout << "car_ptr2," << car_ptr2 << "," << car_ptr1 << "," << reader_head_size << std::endl;
		car_signal_ptr_.push_back(car_ptr1);
		CarSignalInfo1 info1(&raw_data[car_ptr2]);
		car_signal_info1_list_.push_back(info1);
	}
	//歩灯器情報を纏める
	for(uint8_t cou=0; cou<connection_load_count; cou++)
	{
		//uint16_t index = HEADER_SIZE + cou * SIZE_WALKER_SIGNAL_INFO_POINTER + connection_load_count * SIZE_CAR_SIGNAL_INFO_POINTER;
		uint16_t walk_ptr1 = *((uint16_t*)&raw_data[service_ptr + read_raw_data_size_]);//index]);
		buf_rev((uint8_t*)&walk_ptr1, 2);
		read_raw_data_size_ += SIZE_WALKER_SIGNAL_INFO_POINTER;
		if(walk_ptr1 == 0xFFFF) continue;
		uint16_t walk_ptr2 = walk_ptr1 + reader_head_size;
//std::cout << "walk_ptr2," << walk_ptr2 << "," << walk_ptr1 << "," << reader_head_size << std::endl;
		walker_signal_ptr_.push_back(walk_ptr1);
		WalkerSignalInfo1 info1(&raw_data[walk_ptr2]);
		walker_signal_info1_list_.push_back(info1);
	}
}

uint8_t ServiceLoadSignalInfo::getID() const{return id_;}
bool ServiceLoadSignalInfo::getTrafficDirectionInfoFlag() const{return traffic_direction_info_flag_;}
uint8_t ServiceLoadSignalInfo::getTrafficDirectionInfo() const{return traffic_direction_info_;}
std::vector<uint16_t> ServiceLoadSignalInfo::getCarSignalPtr() const{return car_signal_ptr_;}
std::vector<uint16_t> ServiceLoadSignalInfo::getWalkerSignalPtr() const{return walker_signal_ptr_;}
CarSignalInfo1 ServiceLoadSignalInfo::getCarSignalInfo(const size_t index) const{return car_signal_info1_list_[index];}
size_t ServiceLoadSignalInfo::getCarSignalInfoSize() const{return car_signal_info1_list_.size();}
WalkerSignalInfo1 ServiceLoadSignalInfo::getWalkerSignalInfo(const size_t index) const{return walker_signal_info1_list_[index];}
size_t ServiceLoadSignalInfo::getWalkerSignalInfoSize() const{return walker_signal_info1_list_.size();}
size_t ServiceLoadSignalInfo::getServiceSize() const{return read_raw_data_size_;}
}