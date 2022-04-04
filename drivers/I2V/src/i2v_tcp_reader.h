//参考サイト：https://boostjp.github.io/tips/network/tcp.html
#ifndef I2V_TCP_READER
#define I2V_TCP_READER

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cryptography_aes/aes.h>
#include "reader/reader.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/algorithm/string.hpp>
#include <time.h>
#include <std_msgs/Bool.h>
#include <autoware_msgs/GnssDirection.h>
#include <autoware_msgs/GnssSurfaceSpeed.h>
#include <autoware_system_msgs/Date.h>

namespace i2v
{

class I2V_Reader
{
private:
	//const int READ_HZ = 10;

	uint64_t guid_;
	const uint32_t signalID_ = 0x1;
	const uint32_t infomationID_ = 0x1;

	ros::NodeHandle nh_, pnh_;
	ros::Timer timer_;
	//boost::asio::io_service io_;
	//boost::asio::ip::tcp::socket socket_;
	int socket_;
	struct sockaddr_in addr_;
	bool socket_ok_;

	std::string ip_;
	int port_;
	std::vector<uint8_t> aes128_key_;
	std::vector<uint8_t> aes128_constract_vec_;

	void printInit() const;

	void callbackTimer(const ros::TimerEvent& e);
public:
	I2V_Reader(const ros::NodeHandle nh, ros::NodeHandle pnh);

	void release();
	bool isSockOpen();
	std::string getIP();
	int getPort();
};

}
#endif