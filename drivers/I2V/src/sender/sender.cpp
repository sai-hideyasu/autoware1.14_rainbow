#include "sender.h"
#include "../other.h"

namespace i2v
{

void SENDER::clear()
{
	memset(send_buf_, 0, BUF_ALL_SIZE);
}

//通番
void SENDER::setCounter(const uint32_t counter)
{
	
	uint32_t* p = (uint32_t*)&send_buf_[ADD_COUNTER];
	*p = counter;
	buf_rev((uint8_t*)p, 4);
}

uint32_t SENDER::getCounter() const
{
	uint32_t p = *((uint32_t*)&send_buf_[ADD_COUNTER]);
	buf_rev((uint8_t*)&p, 4);
	return p;
}

//発信元車両ID
void SENDER::setGUID(const uint64_t guid_top, const uint64_t guid_bottom)
{
	uint64_t* pbottom = (uint64_t*)&send_buf_[ADD_GUID];
	*pbottom = guid_bottom;
	buf_rev((uint8_t*)pbottom, 8);
	uint64_t* ptop = (uint64_t*)&send_buf_[ADD_GUID+8];
	*ptop = guid_top;
	buf_rev((uint8_t*)ptop, 8);
}

uint64_t SENDER::getGUID_top() const
{
	uint64_t p = *((uint64_t*)&send_buf_[ADD_GUID+8]);
	buf_rev((uint8_t*)&p, 8);
	return p;
}

uint64_t SENDER::getGUID_bottom() const
{
	uint64_t p = *((uint64_t*)&send_buf_[ADD_GUID]);
	buf_rev((uint8_t*)&p, 8);
	return p;
}

//宛先信号機ID
void SENDER::setSignalID(const uint32_t signal_id)
{
	uint32_t* p = (uint32_t*)&send_buf_[ADD_SIGNAL_ID];
	*p = signal_id;
	buf_rev((uint8_t*)p, 4);
}

uint32_t SENDER::getSignalID() const
{
	uint32_t p = *((uint32_t*)&send_buf_[ADD_SIGNAL_ID]);
	buf_rev((uint8_t*)&p, 4);
	return p;
}

//情報種別
void SENDER::setInfomationID(const uint32_t infomation_id)
{
	uint32_t* p = (uint32_t*)&send_buf_[ADD_INFOMATION_ID];
	*p = infomation_id;
	buf_rev((uint8_t*)p, 4);
}

uint32_t SENDER::getInfomationID() const
{
	uint32_t p = *((uint32_t*)&send_buf_[ADD_INFOMATION_ID]);
	buf_rev((uint8_t*)&p, 4);
	return p;
}

//データ作成年
void SENDER::setYear(const uint16_t year)
{
	uint16_t* p = (uint16_t*)&send_buf_[ADD_YEAR];
	*p = year;
	buf_rev((uint8_t*)p, 2);
}

uint16_t SENDER::getYear() const
{
	uint16_t p = *((uint16_t*)&send_buf_[ADD_YEAR]);
	buf_rev((uint8_t*)&p, 2);
	return p;
}

//データ作成月
void SENDER::setMon(const uint8_t mon)
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_MON];
	*p = mon;
}

uint8_t SENDER::getMon() const
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_MON];
	return *p;
}

//データ作成日
void SENDER::setDay(const uint8_t day)
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_DAY];
	*p = day;
}

uint8_t SENDER::getDay() const
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_DAY];
	return *p;
}

//データ作成時
void SENDER::setHour(const uint8_t hour)
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_HOUR];
	*p = hour;
}

uint8_t SENDER::getHour() const
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_HOUR];
	return *p;
}

//データ作成分
void SENDER::setMin(const uint8_t min)
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_MIN];
	*p = min;
}

uint8_t SENDER::getMin() const
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_MIN];
	return *p;
}

//データ作成ミリ秒
void SENDER::setNSec(const uint16_t sec)
{
	uint16_t* p = (uint16_t*)&send_buf_[ADD_NSEC];
	*p = sec;
	buf_rev((uint8_t*)p, 2);
}

uint16_t SENDER::getNSec() const
{
	uint16_t p = *((uint16_t*)&send_buf_[ADD_NSEC]);
	buf_rev((uint8_t*)&p, 2);
	return p;
}

//開始・終了フラグ
void SENDER::setExchangeFlag(const uint8_t flag)
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_EXCHANGE_FLAG];
	*p = flag;
}

uint8_t SENDER::getExchangeFlag() const
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_EXCHANGE_FLAG];
	return *p;
}

//位置情報：緯度
void SENDER::setLatitudeDeg(const double latitude)
{
	uint32_t val = latitude * CONVERT_COEFF_LAT;
	uint32_t* p = (uint32_t*)&send_buf_[ADD_LATITUDE];
	*p = val;
	buf_rev((uint8_t*)p, 4);
}

double SENDER::getLatitudeDeg() const
{
	uint32_t p = *((uint32_t*)&send_buf_[ADD_LATITUDE]);
	buf_rev((uint8_t*)&p, 4);
	double val = (double)p / (double)CONVERT_COEFF_LAT;
	return val;
}

//位置情報：経度
void SENDER::setLongitudeDeg(const double longitude)
{
	uint32_t val = longitude * CONVERT_COEFF_LON;
	uint32_t* p = (uint32_t*)&send_buf_[ADD_LONGITUDE];
	*p = val;
	buf_rev((uint8_t*)p, 4);
}

double SENDER::getLongitudeDeg() const
{
	uint32_t p = *((uint32_t*)&send_buf_[ADD_LONGITUDE]);
	buf_rev((uint8_t*)&p, 4);
	double val = (double)p / (double)CONVERT_COEFF_LON;
	return val;
}

//位置情報：速度
void SENDER::setVelocityPerSec(const double velocity)
{
	uint16_t val = velocity * CONVERT_COEFF_VELOCTY;
	uint16_t* p = (uint16_t*)&send_buf_[ADD_VELOCITY];
	*p = val;
	buf_rev((uint8_t*)p, 2);
}

double SENDER::getVelocityPerSec() const
{
	uint16_t p = *((uint16_t*)&send_buf_[ADD_VELOCITY]);
	buf_rev((uint8_t*)&p, 2);
	double val = (double)p / (double)CONVERT_COEFF_VELOCTY;
	return val;
}

//位置情報：方位角
void SENDER::setAzimuthDeg(const double azimuth)
{
	uint16_t val = azimuth * CONVERT_COEFF_AZIMUTH;
	uint16_t* p = (uint16_t*)&send_buf_[ADD_AZIMUTH];
	*p = val;
	buf_rev((uint8_t*)p, 2);
}

double SENDER::getAzimuthDeg() const
{
	uint16_t p = *((uint16_t*)&send_buf_[ADD_AZIMUTH]);
	buf_rev((uint8_t*)&p, 2);
	double val = (double)p / (double)CONVERT_COEFF_AZIMUTH;
	return val;
}

//将来位置：データ数
void SENDER::setFuturePositionCount(const uint8_t count)
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_FUTURE_POSITION_COUNT];
	*p = count;
}

uint8_t SENDER::getFuturePositionCount() const
{
	uint8_t* p = (uint8_t*)&send_buf_[ADD_FUTURE_POSITION_COUNT];
	return *p;
}

//set系関数で設定したデータを外部に渡す
std::vector<uint8_t> SENDER::getRawDate()
{
	std::vector<uint8_t> raw;
	for(int i=0; i<BUF_ALL_SIZE; i++)
		raw.emplace_back(send_buf_[i]);
	return raw;
}


void SENDER::print() const
{
	std::cout << "通番, " << getCounter() << std::endl;
	std::cout << "発信元車両ID, " << std::hex << getGUID_top() << ", " << getGUID_bottom() << std::endl;
	std::cout << "宛先信号機ID, " << getSignalID() << std::endl;
	std::cout << "情報種別, " << getInfomationID() << std::endl;
	std::cout << "データ作成日時(年), " << std::dec << getYear() << std::endl;
	std::cout << "データ作成日時(月), " << +getMon() << std::endl;
	std::cout << "データ作成日時(日), " << +getDay() << std::endl;
	std::cout << "データ作成日時(時), " << +getHour() << std::endl;
	std::cout << "データ作成日時(分), " << +getMin() << std::endl;
	std::cout << "データ作成日時(ミリ秒), " << getNSec() << std::endl;
	std::cout << "開始・終了フラグ, " << +getExchangeFlag() << std::endl;
	std::cout << "緯度, " << getLatitudeDeg() << std::endl;
	std::cout << "経度, " << getLongitudeDeg() << std::endl;
	std::cout << "速度, " << getVelocityPerSec() << std::endl;
	std::cout << "方位角, " << getAzimuthDeg() << std::endl;
	std::cout << "将来位置数, " << +getFuturePositionCount() << std::endl << std::endl;
}

}