#ifndef I2V_SENDER
#define I2V_SENDER

#include <iostream>
#include <string.h>
#include <vector>

namespace i2v
{

class SENDER
{
private:
	static const size_t SIZE_COUNTER = 4;
	static const size_t SIZE_GUID = 16;
	static const size_t SIZE_SIGNAL_ID = 4;
	static const size_t SIZE_INFOMATION_ID = 4;
	static const size_t SIZE_YEAR = 2;
	static const size_t SIZE_MON = 1;
	static const size_t SIZE_DAY = 1;
	static const size_t SIZE_HOUR = 1;
	static const size_t SIZE_MIN = 1;
	static const size_t SIZE_NSEC = 2;
	static const size_t SIZE_EXCHANGE_FLAG = 1;
	static const size_t SIZE_LATITUDE = 4;
	static const size_t SIZE_LONGITUDE = 4;
	static const size_t SIZE_VELOCITY = 2;
	static const size_t SIZE_AZIMUTH = 2;
	static const size_t SIZE_FUTURE_POSITION_COUNT = 1;

	static const size_t ADD_COUNTER = 0;
	static const size_t ADD_GUID = ADD_COUNTER + SIZE_COUNTER;
	static const size_t ADD_SIGNAL_ID = ADD_GUID + SIZE_GUID;
	static const size_t ADD_INFOMATION_ID = ADD_SIGNAL_ID + SIZE_SIGNAL_ID;
	static const size_t ADD_YEAR = ADD_INFOMATION_ID + SIZE_INFOMATION_ID;
	static const size_t ADD_MON = ADD_YEAR + SIZE_YEAR;
	static const size_t ADD_DAY = ADD_MON + SIZE_MON;
	static const size_t ADD_HOUR = ADD_DAY + SIZE_DAY;
	static const size_t ADD_MIN = ADD_HOUR + SIZE_HOUR;
	static const size_t ADD_NSEC = ADD_MIN + SIZE_MIN;

	static const size_t ADD_EXCHANGE_FLAG = ADD_NSEC + SIZE_NSEC;
	static const size_t ADD_LATITUDE = ADD_EXCHANGE_FLAG + SIZE_EXCHANGE_FLAG;
	static const size_t ADD_LONGITUDE = ADD_LATITUDE + SIZE_LATITUDE;
	static const size_t ADD_VELOCITY = ADD_LONGITUDE + SIZE_LONGITUDE;
	static const size_t ADD_AZIMUTH = ADD_VELOCITY + SIZE_VELOCITY;
	static const size_t ADD_FUTURE_POSITION_COUNT = ADD_AZIMUTH + SIZE_AZIMUTH;

	static const size_t BUF_ALL_SIZE = ADD_FUTURE_POSITION_COUNT + SIZE_FUTURE_POSITION_COUNT;

	static const int CONVERT_COEFF_LAT = (int)(1/(double)0.0000001);
	static const int CONVERT_COEFF_LON = (int)(1/(double)0.0000001);
	static const int CONVERT_COEFF_VELOCTY = (int)(1/(double)0.01);
	static const int CONVERT_COEFF_AZIMUTH = (int)(1/(double)0.0125);

	uint8_t send_buf_[BUF_ALL_SIZE];
public:
	void clear();
	void setCounter(const uint32_t counter);//??????
	uint32_t getCounter() const;
	void setGUID(const uint64_t guid_top, const uint64_t guid_bottom);//???????????????ID
	uint64_t getGUID_top() const;
	uint64_t getGUID_bottom() const;
	void setSignalID(const uint32_t signal_id);//???????????????ID
	uint32_t getSignalID() const;
	void setInfomationID(const uint32_t infomation_id);//????????????
	uint32_t getInfomationID() const;
	void setYear(const uint16_t year);//??????????????????
	uint16_t getYear() const;
	void setMon(const uint8_t mon);//??????????????????
	uint8_t getMon() const;
	void setDay(const uint8_t day);//??????????????????
	uint8_t getDay() const;
	void setHour(const uint8_t hour);//??????????????????
	uint8_t getHour() const;
	void setMin(const uint8_t min);//??????????????????
	uint8_t getMin() const;
	void setNSec(const uint16_t sec);//????????????????????????
	uint16_t getNSec() const;
	void setExchangeFlag(const uint8_t flag);//????????????????????????
	uint8_t getExchangeFlag() const;
	void setLatitudeDeg(const double latitude);//?????????????????????
	double getLatitudeDeg() const;
	void setLongitudeDeg(const double longitude);//?????????????????????
	double getLongitudeDeg() const;
	void setVelocityPerSec(const double velocity);//?????????????????????
	double getVelocityPerSec() const;
	void setAzimuthDeg(const double azimuth);//????????????????????????
	double getAzimuthDeg() const;
	void setFuturePositionCount(const uint8_t count);//???????????????????????????
	uint8_t getFuturePositionCount() const;
	std::vector<uint8_t> getRawDate();//set???????????????????????????????????????????????????

	void print() const;
};

}

#endif