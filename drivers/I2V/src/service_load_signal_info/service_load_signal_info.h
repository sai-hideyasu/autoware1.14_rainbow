#ifndef SERVICE_LOAD_SIGNAL_INFO
#define SERVICE_LOAD_SIGNAL_INFO

#include "../car_signal_info/car_signal_info.h"
#include "../walker_signal_info/walker_signal_info.h"

namespace i2v
{

class ServiceLoadSignalInfo
{
public:
	static const size_t BIT_ID = 8;
	static const size_t BIT_TRAFFIC_DIRECTION_INFO_FLAG = 1;
	static const uint8_t BIT_TRAFFIC_DIRECTION_INFO_FLAG_MASK = 0x80;
	static const size_t BIT_RESERVE = 7;
	static const size_t BIT_TRAFFIC_DIRECTION_INFO = 8;

	static const size_t ADDBIT_ID = 0;
	static const size_t ADDBIT_TRAFFIC_DIRECTION_INFO_FLAG = ADDBIT_ID + BIT_ID;
	static const size_t ADDBIT_RESERVE = ADDBIT_TRAFFIC_DIRECTION_INFO_FLAG + BIT_TRAFFIC_DIRECTION_INFO_FLAG;
	static const size_t ADDBIT_TRAFFIC_DIRECTION_INFO = ADDBIT_RESERVE + BIT_RESERVE;
	static const size_t HEADER_SIZE = (ADDBIT_TRAFFIC_DIRECTION_INFO + BIT_TRAFFIC_DIRECTION_INFO) / 8;

	static const size_t SIZE_CAR_SIGNAL_INFO_POINTER = 16 / 8;
	static const size_t SIZE_WALKER_SIGNAL_INFO_POINTER = 16 / 8;
private:
	uint8_t id_;//方路ID
	bool traffic_direction_info_flag_;//信号通行方向情報有無フラグ
	uint8_t traffic_direction_info_;//信号通行方向情報
	std::vector<uint16_t> car_signal_ptr_;
	std::vector<uint16_t> walker_signal_ptr_;

	std::vector<CarSignalInfo1> car_signal_info1_list_;//このサービス方路の車灯器情報を纏めたもの
	std::vector<WalkerSignalInfo1> walker_signal_info1_list_;//このサービス方路の歩灯器情報を纏めたもの
	size_t read_raw_data_size_;
public:
	//ServiceLoadSignalInfo(const uint8_t* info_raw_data, const uint8_t* all_raw_data, const uint8_t connection_load_count, const uint16_t reader_size);
	ServiceLoadSignalInfo(const uint8_t* raw_data, const uint8_t connection_load_count, const uint16_t service_ptr, const uint16_t reader_head_size);

	uint8_t getID() const;
	bool getTrafficDirectionInfoFlag() const;
	uint8_t getTrafficDirectionInfo() const;
	std::vector<uint16_t> getCarSignalPtr() const;
	std::vector<uint16_t> getWalkerSignalPtr() const;
	CarSignalInfo1 getCarSignalInfo(const size_t index) const;
	size_t getCarSignalInfoSize() const;
	WalkerSignalInfo1 getWalkerSignalInfo(const size_t index) const;
	size_t getWalkerSignalInfoSize() const;
	size_t getServiceSize() const;
};

}

#endif