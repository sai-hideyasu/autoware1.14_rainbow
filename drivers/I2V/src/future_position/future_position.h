#ifndef FUTURE_POSITION
#define FUTURE_POSITION
#include <iostream>

namespace i2v
{

class future_position
{
private:
	uint16_t time_offset_;
	int16_t latitude_offset_;
	int16_t longitude_offset_;
	uint16_t velocity_;
	uint16_t longitude_deg_;
};

}
#endif