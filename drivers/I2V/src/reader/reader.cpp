#include "reader.h"
#include "../other.h"
#include <sstream>
#include <iomanip>

namespace i2v
{

READER::READER(const uint8_t* raw_data)
{
	counter_ = *((uint32_t*)&raw_data[ADD_COUNTER]);//通番
	buf_rev((uint8_t*)&counter_, 4);
	signal_id_ = *((uint32_t*)&raw_data[ADD_SIGNAL_ID]);//送信元信号機ID
	buf_rev((uint8_t*)&signal_id_, 4);
	guid_top_ = *((uint64_t*)&raw_data[ADD_GUID]);//宛先車両ID上
	buf_rev((uint8_t*)&guid_top_, 8);
	guid_bottom_ = *((uint64_t*)&raw_data[ADD_GUID+8]);//宛先車両ID下
	buf_rev((uint8_t*)&guid_bottom_, 8);
	infomation_id_ = *((uint32_t*)&raw_data[ADD_INFOMATION_ID]);//情報種別
	buf_rev((uint8_t*)&infomation_id_, 4);
	year_ = *((uint16_t*)&raw_data[ADD_YEAR]);//判定日時年
	buf_rev((uint8_t*)&year_, 2);
	mon_ = *((uint8_t*)&raw_data[ADD_MON]);//判定日時月
	day_ = *((uint8_t*)&raw_data[ADD_DAY]);//判定日時日
	hour_ = *((uint8_t*)&raw_data[ADD_HOUR]);//判定日時時
	min_ = *((uint8_t*)&raw_data[ADD_MIN]);//判定日時分
	nsec_ = *((uint16_t*)&raw_data[ADD_NSEC]);//判定日時ミリ秒
	buf_rev((uint8_t*)&nsec_, 2);

	offer_prefecture_code_ = *((uint8_t*)&raw_data[ADDBIT_OFFER_PREFECTURE_CODE / 8]);//提供点管理番号：都道府県コード
	offer_point_code_ = (raw_data[ADDBIT_OFFER_POINT_CODE / 8] & BIT_OFFER_POINT_CODE_MASK) >> BIT_OFFER_POINT_CODE_SHIFT;//提供点管理番号：提供点種別コード
	uint8_t uint8_2[2];
	uint8_2[0] = raw_data[ADDBIT_OFFER_POINT_CODE / 8+1];
	uint8_2[1] = raw_data[ADDBIT_OFFER_POINT_CODE / 8+0];
	uint8_2[1] &= BIT_OFFER_LOAD_ID_MASK;
	offer_load_id_ = *((uint16_t*)uint8_2);//提供点管理番号：交差点ID/単路ID
	offer_version_num_stander_ = *((uint8_t*)&raw_data[ADDBIT_OFFER_VERSION_NUM_STANDARD / 8]);//提供点管理番号：バージョン情報(規格)
	offer_version_num_definition_ = *((uint8_t*)&raw_data[ADDBIT_OFFER_VERSION_NUM_DEFINITION / 8]);//提供点管理番号：バージョン情報(定義情報)

	date_year_ = *((uint8_t*)&raw_data[ADDBIT_DATE_YEAR / 8]);//作成日時年
	date_mon_ = *((uint8_t*)&raw_data[ADDBIT_DATE_MON / 8]);//作成日時月
	date_day_ = *((uint8_t*)&raw_data[ADDBIT_DATE_DAY / 8]);//作成日時日
	date_hour_ = *((uint8_t*)&raw_data[ADDBIT_DATE_HOUR / 8]);//作成日時時
	date_min_ = *((uint8_t*)&raw_data[ADDBIT_DATE_MIN / 8]);//作成日時分
	date_sec_ = *((uint8_t*)&raw_data[ADDBIT_DATE_SEC / 8]);//作成日時病
	date_sec10m_ = *((uint8_t*)&raw_data[ADDBIT_DATE_SEC_10M / 8]);//作成日時ミリ秒

	signal_situation_ = *((uint8_t*)&raw_data[ADDBIT_SIGNAL_SITUATION / 8]);//信号状態情報
	identification_ = *((uint8_t*)&raw_data[ADDBIT_IDENTIFICATION / 8]);//特定制御動作中フラグ
	system_info_ = *((uint8_t*)&raw_data[ADDBIT_SYSTEM_INFO / 8]);//システム状態
	event_count_ = *((uint8_t*)&raw_data[ADDBIT_EVENT_COUNT / 8]);//イベントカウンタ
	car_signal_count_ = *((uint8_t*)&raw_data[ADDBIT_CAR_SIGNAL_COUNT / 8]);//車灯器数
	walker_signal_count_ = *((uint8_t*)&raw_data[ADDBIT_CAR_SIGNAL_COUNT / 8]);//歩灯器数
	connection_load_count_ = *((uint8_t*)&raw_data[ADDBIT_CONNECTION_LOAD_COUNT / 8]);//接続方路数(I)
	service_load_count_ = *((uint8_t*)&raw_data[ADDBIT_SERVICE_LOAD_COUNT / 8]);//サービス方路数(J)

	size_t service_ptr = ADD_FIRST_SERVICE_LOAD_SIGNAL_INFO;
	for(int cou=0; cou<service_load_count_; cou++)
	{
		i2v::ServiceLoadSignalInfo info(raw_data, connection_load_count_, service_ptr, HEADER_SIZE);
		service_ptr += info.getServiceSize();
		service_info_list_.push_back(info);
	}
}

void READER::print() const
{
	std::cout << "通番," << counter_ << std::endl;
	std::cout << std::showbase << std::hex;
	std::cout << "送信元 信号機ID, " << signal_id_ << std::endl;
	std::cout << "宛先 車両ID(top), " << guid_top_ << std::endl;
	std::cout << "宛先 車両ID(bottom), " << guid_bottom_ << std::endl;
	std::cout << "情報種別, " << infomation_id_ << std::endl;
	std::cout << std::dec;
	std::cout << "判定日時, " << +year_ << "/" << +mon_ << "/" << +day_ << "-" << +hour_ << "/" << +min_ << "/" << nsec_ / (double)1000.0 << std::endl;
	std::cout << "提供点管理番号：都道府県コード, " << +offer_prefecture_code_ << std::endl;
	std::cout << "提供点管理番号：提供点種別コード, " << +offer_point_code_ << std::endl;
	std::cout << std::hex;
	std::cout << "提供点管理番号：交差点ID/単路ID, " << offer_load_id_ << std::endl;
	std::cout << "提供点管理番号：バージョン番号(規格), " << +offer_version_num_stander_ << std::endl;
	std::cout << "提供点管理番号：バージョン番号(定義情報), " << +offer_version_num_definition_ << std::endl;
	std::cout << std::noshowbase;
	std::cout << "作成日時, " << +date_year_ << "/" << +date_mon_ << "/" << +date_day_ << "-" << +date_hour_ << "/" << +date_min_ << "/" << +date_sec_ << "." << +date_sec10m_ << std::endl;
	std::cout << std::showbase;
	std::cout << "信号状態情報," << +signal_situation_ << std::endl;
	std::cout << "特定制御動作中フラグ," << +identification_ << std::endl;
	std::cout << "システム状態," << +system_info_ << std::endl;
	std::cout << "イベントカウンタ," << +event_count_ << std::endl;
	std::cout << std::dec;
	std::cout << "車灯器数," << +car_signal_count_ << std::endl;
	std::cout << "歩灯器数," << +walker_signal_count_ << std::endl;
	std::cout << "接続方路数," << +connection_load_count_ << std::endl;
	std::cout << "サービス方路数," << +service_load_count_ << std::endl;

	/*for(size_t sercou=0; sercou<service_info_list_.size(); sercou++)
	{
		i2v::ServiceLoadSignalInfo service_info = service_info_list_[sercou];
		std::cout << "サービス方路信号情報" << sercou+1 << "：方路ID, " << +service_info.getID() << std::endl;
		std::cout << "サービス方路信号情報" << sercou+1 << "：信号通行方向情報有無フラグ, " << std::boolalpha << service_info.getTrafficDirectionInfoFlag() << std::endl;
		std::cout << "サービス方路信号情報" << sercou+1 << "：信号通行方向情報, " << +service_info.getTrafficDirectionInfo() << std::endl;

		for(size_t carcou1=0; carcou1<service_info.getCarSignalInfoSize(); carcou1++)
		{
			i2v::CarSignalInfo1 car_info1 = service_info.getCarSignalInfo(carcou1);
			std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：車灯器ID," << +car_info1.getID() << std::endl;
			std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：灯色出力変化数," << +car_info1.getOutputChangeCount() << std::endl;
		}
	}*/

	for(size_t sercou=0; sercou<service_info_list_.size(); sercou++)
	{
		i2v::ServiceLoadSignalInfo service_info = service_info_list_[sercou];
		std::cout << "サービス方路信号情報" << sercou+1 << "：方路ID, " << +service_info.getID() << std::endl;
		std::cout << "サービス方路信号情報" << sercou+1 << "：信号通行方向情報有無フラグ, " << std::boolalpha << service_info.getTrafficDirectionInfoFlag() << std::endl;
		std::cout << "サービス方路信号情報" << sercou+1 << "：信号通行方向情報, " << +service_info.getTrafficDirectionInfo() << std::endl;

		for(size_t carcou1=0; carcou1<service_info.getCarSignalPtr().size(); carcou1++)
		{
			std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：車灯器pointer," << std::hex << service_info.getCarSignalPtr()[carcou1] << std::dec << std::endl;
			i2v::CarSignalInfo1 car_info1 = service_info.getCarSignalInfo(carcou1);
			std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：車灯器ID," << +car_info1.getID() << std::endl;
			std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：灯色出力変化数," << +car_info1.getOutputChangeCount() << std::endl;

			for(uint8_t carcou2=0; carcou2<car_info1.getOutputChangeCount(); carcou2++)
			{
				i2v::CarSignalInfo2 car_info2 = car_info1.getInfo2(carcou2);
				std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：車両灯器情報" << +(carcou2+1) << "：丸信号灯色表示, " << std::hex << +car_info2.getCircleColorDisplay()<< std::dec << std::endl;
				std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：車両灯器情報" << +(carcou2+1) << "：青矢信号表示方向, " << +car_info2.getArrowColorDisplay() << std::endl;
				std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：車両灯器情報" << +(carcou2+1) << "：カウントダウン停止フラグ, " << std::boolalpha << car_info2.getCountdownStopFlag() << std::endl;
				std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：車両灯器情報" << +(carcou2+1) << "：最小残秒数, " << (double)(car_info2.getMinRemainingSec10m()/(double)10.0) << std::endl;
				std::cout << "サービス方路信号情報" << sercou+1 << "：車灯器情報" << carcou1+1 << "：車両灯器情報" << +(carcou2+1) << "：最大残秒数, " << (double)(car_info2.getMaxRemainingSec10m()/(double)10.0) << std::endl;
			}
		}

		for(size_t walkcou1=0; walkcou1<service_info.getWalkerSignalInfoSize(); walkcou1++)
		{
			std::cout << "サービス方路信号情報" << sercou+1 << "：歩灯器情報" << walkcou1+1 << "：歩灯器pointer," << std::hex << service_info.getWalkerSignalPtr()[walkcou1] << std::dec << std::endl;
			i2v::WalkerSignalInfo1 walker_info1 = service_info.getWalkerSignalInfo(walkcou1);
			std::cout << "サービス方路信号情報" << sercou+1 << "：歩灯器情報" << walkcou1+1 << "：歩灯器ID," << +walker_info1.getID() << std::endl;
			std::cout << "サービス方路信号情報" << sercou+1 << "：歩灯器情報" << walkcou1+1 << "：灯色出力変化数," << +walker_info1.getOutputChangeCount() << std::endl;
			for(uint8_t walkcou2=0; walkcou2<walker_info1.getOutputChangeCount(); walkcou2++)
			{
				i2v::WalkerSignalInfo2 walker_info2 = walker_info1.getInfo2(walkcou2);
				std::cout << "サービス方路信号情報" << sercou+1 << "：歩灯器情報" << walkcou1+1 << "：歩行者灯器情報" << +(walkcou2+1) << "：歩行者信号灯色表示, " << +walker_info2.getCircleColorDisplay()<< std::endl;
				std::cout << "サービス方路信号情報" << sercou+1 << "：歩灯器情報" << walkcou1+1 << "：歩行者灯器情報" << +(walkcou2+1) << "：カウントダウン停止フラグ, " << std::boolalpha << walker_info2.getCountdownStopFlag() << std::endl;
				std::cout << "サービス方路信号情報" << sercou+1 << "：歩灯器情報" << walkcou1+1 << "：歩行者灯器情報" << +(walkcou2+1) << "：最小残秒数, " << (double)(walker_info2.getMinRemainingSec10m()/(double)10.0) << std::endl;
				std::cout << "サービス方路信号情報" << sercou+1 << "：歩灯器情報" << walkcou1+1 << "：歩行者灯器情報" << +(walkcou2+1) << "：最大残秒数, " << (double)(walker_info2.getMaxRemainingSec10m()/(double)10.0) << std::endl;
			}
		}
	}

	std::cout << std::endl;
}

size_t READER::getServeceInfoSize() const{return service_info_list_.size();}
ServiceLoadSignalInfo READER::getServiceInfo(const size_t index) const{return service_info_list_[index];}

std::string READER::recieveDateStrig() const
{
	std::stringstream ss;
	ss << std::setfill('0') << std::setw(4) << year_ << "/" << std::setw(2) << +mon_ << "/" << +day_ << "-" << +hour_ << ":" << +min_ << ":" << nsec_ / (double)1000.0;
	return ss.str();
}

std::string READER::sendDateString() const
{
	std::stringstream ss;
	ss << std::hex << std::setfill('0') << std::setw(4) << +date_year_ << "/" << std::setw(2) << +date_mon_ << "/" << +date_day_ << "-" << +date_hour_ << ":" << +date_min_ << ":" << +date_sec_ << "." << +date_sec10m_;
	return ss.str();
}

}