#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <time.h>

class SENDER
{
private:
	const size_t SIZE_COUNTER = 4;
	const size_t SIZE_GUID = 16;
	const size_t SIZE_SIGNAL_ID = 4;
	const size_t SIZE_INFOMATION_ID = 4;
	const size_t SIZE_YEAR = 2;
	const size_t SIZE_MON = 1;
	const size_t SIZE_DAY = 1;
	const size_t SIZE_HOUR = 1;
	const size_t SIZE_MIN = 1;
	const size_t SIZE_SEC = 2;

	const size_t ADD_COUNTER = 0;
	const size_t ADD_GUID = ADD_COUNTER + SIZE_COUNTER;
	const size_t ADD_SIGNAL_ID = ADD_GUID + SIZE_GUID;
	const size_t ADD_INFOMATION_ID = ADD_SIGNAL_ID + SIZE_SIGNAL_ID;
	const size_t ADD_YEAR = ADD_INFOMATION_ID + SIZE_INFOMATION_ID;
	const size_t ADD_MON = ADD_YEAR + SIZE_YEAR;
	const size_t ADD_DAY = ADD_MON + SIZE_MON;
	const size_t ADD_HOUR = ADD_DAY + SIZE_DAY;
	const size_t ADD_MIN = ADD_HOUR + SIZE_HOUR;
	const size_t ADD_SEC = ADD_MIN + SIZE_MIN;

	const size_t BUF_ALL_SIZE = 50;
	unsigned char send_buf_[50];
public:
	void clear()
	{
		memset(send_buf_, 0, BUF_ALL_SIZE);
	}
	void setCounter(const uint32_t counter)
	{
		uint32_t* p = (uint32_t*)&send_buf_[ADD_COUNTER];
		*p = counter;
	}
	void setGUID(const uint64_t guid)
	{
		uint64_t* p = (uint64_t*)&send_buf_[ADD_GUID];
		*p = guid;
	}
	void setSignalID(const uint32_t signal_id)
	{
		uint32_t* p = (uint32_t*)&send_buf_[ADD_SIGNAL_ID];
		*p = signal_id;
	}
	void setInfomationID(const uint32_t infomation_id)
	{
		uint32_t* p = (uint32_t*)&send_buf_[ADD_INFOMATION_ID];
		*p = infomation_id;
	}
	void setYear(const uint16_t year)
	{
		uint16_t* p = (uint16_t*)&send_buf_[ADD_YEAR];
		*p = year;
	}
	void setMon(const uint8_t mon)
	{
		uint8_t* p = (uint8_t*)&send_buf_[ADD_MON];
		*p = mon;
	}
	void setDay(const uint8_t day)
	{
		uint8_t* p = (uint8_t*)&send_buf_[ADD_DAY];
		*p = day;
	}
	void setHour(const uint8_t hour)
	{
		uint8_t* p = (uint8_t*)&send_buf_[ADD_HOUR];
		*p = hour;
	}
	void setMin(const uint8_t min)
	{
		uint8_t* p = (uint8_t*)&send_buf_[ADD_MIN];
		*p = min;
	}
	void setSec(const uint16_t sec)
	{
		uint16_t* p = (uint16_t*)&send_buf_[ADD_SEC];
		*p = sec;
	}
};

int main(int argc, char** argv)
{
	/*{
		struct timeval get_time_ret;
		gettimeofday(&get_time_ret, NULL);
		time_t nowtime = (get_time_ret.tv_sec * 1000) + (get_time_ret.tv_usec / 1000);
		struct tm* time_struct = localtime(&get_time_ret.tv_sec);
		mktime(time_struct);
		std::cout << time_struct->tm_year+1900 << "/" << time_struct->tm_mon+1 << "/" << time_struct->tm_mday << "-" << time_struct->tm_hour << "/" << time_struct->tm_min << "/" << time_struct->tm_sec*1000 + get_time_ret.tv_usec/1000 << std::endl;
	}*/
	// initialize node
	ros::init(argc,argv,"i2v_tcp_send");
	ros::NodeHandle nh, pnh("~");

	const uint64_t carID = 5963;
	const uint32_t signalID = 0x1;
	const uint32_t infomationID = 0x1;

	// setup server infomation
	/*std::string ip_str;
	int port;
	pnh.getParam("ip", ip_str);
	pnh.getParam("port", port);

	// connect server PortNo=3001
	boost::asio::io_service io;
	boost::asio::ip::tcp::socket sock(io);
	boost::asio::ip::tcp::endpoint endpoint = boost::asio::ip::tcp::endpoint{boost::asio::ip::address::from_string(ip_str), port};
	sock.connect(endpoint);

	if(!sock.is_open())
	{
		std::cout << "not open" << std::endl;
		return 0;
	}*/

	ros::Rate rate(30);
	uint32_t loop_count = 0;
	SENDER sender;
	while(ros::ok())
	{
		ros::spinOnce();
		sender.clear();
		sender.setCounter(loop_count);
		sender.setGUID(carID);
		sender.setSignalID(signalID);
		sender.setInfomationID(infomationID);

		struct timeval get_time_ret;
		gettimeofday(&get_time_ret, NULL);
		time_t nowtime = (get_time_ret.tv_sec * 1000) + (get_time_ret.tv_usec / 1000);
		struct tm* time_struct = localtime(&get_time_ret.tv_sec);
		//mktime(time_struct);
		//std::cout << time_struct->tm_year+1900 << "/" << time_struct->tm_mon+1 << "/" << time_struct->tm_mday << "-" << time_struct->tm_hour << "/" << time_struct->tm_min << "/" << time_struct->tm_sec*1000 + get_time_ret.tv_usec/1000 << std::endl;

		sender.setYear((uint16_t)time_struct->tm_year);
		sender.setMon((uint8_t)time_struct->tm_mon+1);
		sender.setDay((uint8_t)time_struct->tm_mday);
		sender.setHour((uint8_t)time_struct->tm_hour);
		sender.setMin((uint8_t)time_struct->tm_min);
		sender.setSec((uint8_t)time_struct->tm_sec*1000 + get_time_ret.tv_usec/1000);
		loop_count++;
		rate.sleep();
	}
	//sock.close();
	return 0;
}

