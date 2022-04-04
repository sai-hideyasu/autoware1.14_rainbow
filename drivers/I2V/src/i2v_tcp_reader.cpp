//参考サイト：https://boostjp.github.io/tips/network/tcp.html
#include "i2v_tcp_reader.h"

/*void dump(char *msg, uint8_t *d, int len)
{
	fprintf(stderr, "%s (%2d) : ", msg, len);
	for (int i = 0; i < len; i++)
	{
		if (i % 10 == 0 && i != 0)
			fprintf(stderr, "| ");
		fprintf(stderr, "%02X ", d[i]);
	}
	fprintf(stderr, "\n");
}*/

namespace i2v
{

I2V_Reader::I2V_Reader(const ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		//, io_()
		//, socket_(io_)
{
	guid_ = pnh_.param<int>("guid", 5963);
	ip_ = pnh_.param<std::string>("ip", "210.156.173.53");
	port_ = pnh_.param("port", 56974);
	aes128_key_ = string2vector(pnh_.param<std::string>("evp_key", "FcpE4rN66Rn2bx9A"));
	aes128_constract_vec_ = string2vector(pnh_.param<std::string>("evp_constract_vec", "8LdC4iuDsvvTv6CB"));

	printInit();

	// connect server
	/*boost::system::error_code error;
	boost::asio::ip::tcp::endpoint endpoint = boost::asio::ip::tcp::endpoint{boost::asio::ip::address::from_string(ip_), port_};
	socket_.connect(endpoint, error);
	if (error) {
		std::cout << "接続失敗(受信) : " << error.message() << std::endl;
	}
	else {
		std::cout << "接続成功(受信)" << std::endl;
	}*/
	socket_ = socket(AF_INET, SOCK_DGRAM, 0);
	addr_.sin_family = AF_INET;
	addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
	addr_.sin_port = htons(port_); 
	if(socket_ == -1){
		std::cout << "接続失敗(送信) : " << std::endl;
	}
	else {
		std::cout << "接続成功(送信)" << std::endl;
	}
	if(bind(socket_, (const struct sockaddr *)&addr_, sizeof(addr_)) == -1){
		std::cout << "bind失敗(送信) : " << std::endl; socket_ok_ = false;
	}
	else {
		std::cout << "bind成功(送信)" << std::endl; socket_ok_ = true;
	}
	timer_ = nh_.createTimer(ros::Duration(1.0/READ_HZ), &I2V_Reader::callbackTimer, this);
}

void I2V_Reader::printInit() const
{
	std::cout << "GUID," << guid_ << std::endl;
	std::cout << "IP ADDRESS," << ip_ << std::endl;
	std::cout << "PORT_," << port_ << std::endl;
	std::cout << "AES128_KEY," << vector2string(aes128_key_) << std::endl;
	std::cout << "AES128_CONSTRACT_VEC," << vector2string(aes128_constract_vec_) << std::endl;
	std::cout << std::endl;
}

void I2V_Reader::callbackTimer(const ros::TimerEvent& e)
{
	/*boost::asio::streambuf sbuf;
	boost::system::error_code error;
	std::vector<uint8_t> dec_data;
	size_t read_size = boost::asio::read(socket_, sbuf, boost::asio::transfer_all(), error);

	if(error && error != boost::asio::error::eof)
	{
		std::cout << "受信失敗: " << error.message() << std::endl;
		return;
	}
	else
	{
		std::cout << "受信成功" << std::endl;
	}*/
	const size_t BUF_SIZE = 500;
	uint8_t sbuf[BUF_SIZE];
	memset(sbuf, 0, BUF_SIZE);
	ssize_t read_size = recv(socket_, sbuf, BUF_SIZE, 0);
	if(read_size == -1)
	{
		std::cout << "受信失敗: " << std::endl;
		return;
	}
	else
	{
		std::cout << "受信成功" << std::endl;
	}

	const uint8_t* rawdata = sbuf;//boost::asio::buffer_cast<const uint8_t*>(sbuf.data());
	std::vector<uint8_t> rawvec;
	for(int i=0; i<read_size; i++) rawvec.push_back(rawdata[i]);
	cryptography_aes::AES aes128(aes128_key_, aes128_constract_vec_);
	std::vector<uint8_t> decode_data;
	int decode_error = aes128.decode(rawvec, decode_data);
	if(decode_error != cryptography_aes::AES::OK)
	{
		std::cout << "デコード失敗" << decode_error << std::endl;
		return;
	}
	else
	{
		std::cout << "デコード成功" << std::endl;
	}

	i2v::READER reader(decode_data.data());
	reader.print();
}

void I2V_Reader::release(){if(isSockOpen()) close(socket_);}
bool I2V_Reader::isSockOpen() {return socket_ok_;}
std::string I2V_Reader::getIP() {return ip_;}
int I2V_Reader::getPort() {return port_;}

}