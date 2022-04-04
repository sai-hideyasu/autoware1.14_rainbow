#ifndef I2V_READER
#define I2V_READER

#include "../service_load_signal_info/service_load_signal_info.h"
#include <iostream>
#include <vector>

namespace i2v
{

class READER
{
public:
	static const size_t SIZE_COUNTER = 4;
	static const size_t SIZE_SIGNAL_ID = 4;
	static const size_t SIZE_GUID = 16;
	static const size_t SIZE_INFOMATION_ID = 4;
	static const size_t SIZE_YEAR = 2;
	static const size_t SIZE_MON = 1;
	static const size_t SIZE_DAY = 1;
	static const size_t SIZE_HOUR = 1;
	static const size_t SIZE_MIN = 1;
	static const size_t SIZE_NSEC = 2;

	static const size_t ADD_COUNTER = 0;
	static const size_t ADD_SIGNAL_ID = ADD_COUNTER + SIZE_COUNTER;
	static const size_t ADD_GUID = ADD_SIGNAL_ID + SIZE_SIGNAL_ID;
	static const size_t ADD_INFOMATION_ID = ADD_GUID + SIZE_GUID;
	static const size_t ADD_YEAR = ADD_INFOMATION_ID + SIZE_INFOMATION_ID;
	static const size_t ADD_MON = ADD_YEAR + SIZE_YEAR;
	static const size_t ADD_DAY = ADD_MON + SIZE_MON;
	static const size_t ADD_HOUR = ADD_DAY + SIZE_DAY;
	static const size_t ADD_MIN = ADD_HOUR + SIZE_HOUR;
	static const size_t ADD_NSEC = ADD_MIN + SIZE_MIN;

	static const size_t HEADER_SIZE = ADD_NSEC + SIZE_NSEC;

	static const size_t BIT_OFFER_PREFECTURE_CODE = 8;
	static const size_t BIT_OFFER_POINT_CODE = 1;
	static const uint8_t BIT_OFFER_POINT_CODE_MASK = 0x80;
	static const uint8_t BIT_OFFER_POINT_CODE_SHIFT = 7;
	static const size_t BIT_OFFER_LOAD_ID = 15;
	static const uint8_t BIT_OFFER_LOAD_ID_MASK = 0x7F;
	static const size_t BIT_OFFER_RESERVE1 = 8;
	static const size_t BIT_OFFER_RESERVE2 = 8;
	static const size_t BIT_OFFER_VERSION_NUM_STANDARD = 8;
	static const size_t BIT_OFFER_VERSION_NUM_DEFINITION = 8;

	static const size_t ADDBIT_OFFER_PREFECTURE_CODE = HEADER_SIZE * 8;
	static const size_t ADDBIT_OFFER_POINT_CODE = ADDBIT_OFFER_PREFECTURE_CODE + BIT_OFFER_PREFECTURE_CODE;
	static const size_t ADDBIT_OFFER_LOAD_ID = ADDBIT_OFFER_POINT_CODE + BIT_OFFER_POINT_CODE;
	static const size_t ADDBIT_OFFER_RESERVE1 = ADDBIT_OFFER_LOAD_ID + BIT_OFFER_LOAD_ID;
	static const size_t ADDBIT_OFFER_RESERVE2 = ADDBIT_OFFER_RESERVE1 + BIT_OFFER_RESERVE1;
	static const size_t ADDBIT_OFFER_VERSION_NUM_STANDARD = ADDBIT_OFFER_RESERVE2 + BIT_OFFER_RESERVE2;
	static const size_t ADDBIT_OFFER_VERSION_NUM_DEFINITION = ADDBIT_OFFER_VERSION_NUM_STANDARD + BIT_OFFER_VERSION_NUM_STANDARD;

	static const size_t BIT_RESERVE1 = 8;
	static const size_t BIT_RESERVE2 = 8;

	static const size_t ADDBIT_RESERVE1 = ADDBIT_OFFER_VERSION_NUM_DEFINITION + BIT_OFFER_VERSION_NUM_DEFINITION;
	static const size_t ADDBIT_RESERVE2 = ADDBIT_RESERVE1 + BIT_RESERVE1;

	static const size_t BIT_DATE_YEAR = 8;
	static const size_t BIT_DATE_MON = 8;
	static const size_t BIT_DATE_DAY = 8;
	static const size_t BIT_DATE_HOUR = 8;
	static const size_t BIT_DATE_MIN = 8;
	static const size_t BIT_DATE_SEC = 8;
	static const size_t BIT_DATE_SEC_10M = 8;

	static const size_t ADDBIT_DATE_YEAR = ADDBIT_RESERVE2 + BIT_RESERVE2;
	static const size_t ADDBIT_DATE_MON = ADDBIT_DATE_YEAR + BIT_DATE_YEAR;
	static const size_t ADDBIT_DATE_DAY = ADDBIT_DATE_MON + BIT_DATE_MON;
	static const size_t ADDBIT_DATE_HOUR = ADDBIT_DATE_DAY + BIT_DATE_DAY;
	static const size_t ADDBIT_DATE_MIN = ADDBIT_DATE_HOUR + BIT_DATE_HOUR;
	static const size_t ADDBIT_DATE_SEC = ADDBIT_DATE_MIN + BIT_DATE_MIN;
	static const size_t ADDBIT_DATE_SEC_10M = ADDBIT_DATE_SEC + BIT_DATE_SEC;

	static const size_t BIT_SIGNAL_SITUATION = 8;
	static const size_t BIT_IDENTIFICATION = 8;
	static const size_t BIT_SYSTEM_INFO = 8;
	static const size_t BIT_EVENT_COUNT = 8;
	static const size_t BIT_CAR_SIGNAL_COUNT = 8;
	static const size_t BIT_WALKER_SIGNAL_COUNT = 8;
	static const size_t BIT_CONNECTION_LOAD_COUNT = 8;
	static const size_t BIT_SERVICE_LOAD_COUNT = 8;

	static const size_t ADDBIT_SIGNAL_SITUATION = ADDBIT_DATE_SEC_10M + BIT_DATE_SEC_10M;
	static const size_t ADDBIT_IDENTIFICATION = ADDBIT_SIGNAL_SITUATION + BIT_SIGNAL_SITUATION;
	static const size_t ADDBIT_SYSTEM_INFO = ADDBIT_IDENTIFICATION + BIT_IDENTIFICATION;
	static const size_t ADDBIT_EVENT_COUNT = ADDBIT_SYSTEM_INFO + BIT_SYSTEM_INFO;
	static const size_t ADDBIT_CAR_SIGNAL_COUNT = ADDBIT_EVENT_COUNT + BIT_EVENT_COUNT;
	static const size_t ADDBIT_WALKER_SIGNAL_COUNT = ADDBIT_CAR_SIGNAL_COUNT + BIT_CAR_SIGNAL_COUNT;
	static const size_t ADDBIT_CONNECTION_LOAD_COUNT = ADDBIT_WALKER_SIGNAL_COUNT + BIT_WALKER_SIGNAL_COUNT;
	static const size_t ADDBIT_SERVICE_LOAD_COUNT = ADDBIT_CONNECTION_LOAD_COUNT + BIT_CONNECTION_LOAD_COUNT;

	static const size_t ADD_FIRST_SERVICE_LOAD_SIGNAL_INFO = (ADDBIT_SERVICE_LOAD_COUNT + BIT_SERVICE_LOAD_COUNT) / 8;
private:
	uint32_t counter_;//通番
	uint32_t signal_id_;//送信元信号機ID
	uint64_t guid_top_;//宛先車両ID上
	uint64_t guid_bottom_;//宛先車両ID下
	uint32_t infomation_id_;//情報種別
	uint16_t year_;//判定日時年
	uint8_t mon_;//判定日時月
	uint8_t day_;//判定日時日
	uint8_t hour_;//判定日時時
	uint8_t min_;//判定日時分
	uint16_t nsec_;//判定日時ミリ秒

	uint8_t offer_prefecture_code_;//提供点管理番号：都道府県コード
	uint8_t offer_point_code_;//提供点管理番号：提供点種別コード
	uint16_t offer_load_id_;//提供点管理番号：交差点ID/単路ID
	uint8_t offer_version_num_stander_;//提供点管理番号：バージョン情報(規格)
	uint8_t offer_version_num_definition_;//提供点管理番号：バージョン情報(定義情報)

	uint8_t date_year_;//作成日時年
	uint8_t date_mon_;//作成日時月
	uint8_t date_day_;//作成日時日
	uint8_t date_hour_;//作成日時時
	uint8_t date_min_;//作成日時分
	uint8_t date_sec_;//作成日時病
	uint8_t date_sec10m_;//作成日時１０ミリ秒

	uint8_t signal_situation_;//信号状態情報
	uint8_t identification_;//特定制御動作中フラグ
	uint8_t system_info_;//システム状態
	uint8_t event_count_;//イベントカウンタ
	uint8_t car_signal_count_;//車灯器数
	uint8_t walker_signal_count_;//歩灯器数
	uint8_t connection_load_count_;//接続方路数(I)
	uint8_t service_load_count_;//サービス方路数(J)

	std::vector<i2v::ServiceLoadSignalInfo> service_info_list_;//サービス方路信号情報を纏めたもの
public:
	READER(const uint8_t* raw_data);

	void print() const;

	size_t getServeceInfoSize() const;
	ServiceLoadSignalInfo READER::getServiceInfo(const size_t index) const;
	std::string recieveDateStrig() const;
	std::string sendDateString() const;
};

}

#endif