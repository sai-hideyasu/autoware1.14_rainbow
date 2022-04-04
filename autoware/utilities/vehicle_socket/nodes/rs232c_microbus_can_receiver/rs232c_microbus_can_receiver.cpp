#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <autoware_can_msgs/MicroBusSHHV.h>
#include <autoware_can_msgs/MicroBusSHHVArrange.h>
#include <autoware_can_msgs/MicroBusSPPM.h>

std::vector<std::string> split(const std::string &string, const char sep)
{
	std::vector<std::string> str_vec_ptr;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, sep))
		str_vec_ptr.push_back(token);

	return str_vec_ptr;
}

class ReadData
{
public:
	static const size_t DATA_MAX_SIZE = 110;
	static const size_t PARAMS_COUNT = 16;

	static const unsigned short JOY_CENTER_VOLTAGE = 1024;

	static const int DATA_TYPE_NOT = 0;
	static const int DATA_TYPE_SHHV = 1;
	static const int DATA_TYPE_SPPM = 2;
private:
	std::vector<unsigned char> read_data_;
	size_t read_size_;
	autoware_can_msgs::MicroBusSHHV shhv_;
	autoware_can_msgs::MicroBusSHHVArrange shhv_arrange_;
	autoware_can_msgs::MicroBusSPPM sppm_;
	int data_type_;

	int stoi_sgn(std::string str)
	{
		if(str.c_str()[0] == '-')
		{
			return std::stoi(&str.c_str()[1]) * -1;
		}
		else
			return std::stoi(str);
	}
public:
	ReadData()
		: read_size_(0)
		, data_type_(DATA_TYPE_NOT)
	{

	}

	void addData(unsigned char buf[], size_t add_size)
	{
		data_type_ = DATA_TYPE_NOT;

		for(unsigned int i=0; i<add_size; i++)
		{
			read_data_.push_back(buf[i]);
			if(buf[i] == 'x')
			{
				if(read_data_.size() == DATA_MAX_SIZE)
				{
					char buf[DATA_MAX_SIZE+1];
					memcpy(buf, read_data_.data(), read_data_.size());

					buf[DATA_MAX_SIZE] = '\0';
					//std::cout << read_data_.size() << std::endl;
					//std::cout << buf << std::endl;
					std::vector<std::string> params = split(std::string(buf), ',');
					//std::cout << params.size() << std::endl;
					if(params.size() == PARAMS_COUNT)
					{
						std::vector<std::string> str_code = split(params[0], ' ');
						if(str_code[0] == "shhv")//ステア
						{
							shhv_.status_code = stoi_sgn(str_code[str_code.size()-1]);
							shhv_.joy_angle_main = stoi_sgn(params[1]);
							shhv_.mechanism_steering_main = stoi_sgn(params[2]);
							shhv_.motor_applied_voltage = stoi_sgn(params[3]);
							shhv_.motor_angular_velocity = stoi_sgn(params[4]);
							shhv_.motor_electric_current = stoi_sgn(params[5]);
							shhv_.pulsu_count_velocity = stoi_sgn(params[6]);
							shhv_.power_supply_voltage = stoi_sgn(params[7]);
							shhv_.steer_limit_rate = stoi_sgn(params[8]);
							shhv_.mecha_offset_adjusted_value = stoi_sgn(params[9]);
							shhv_.joy_fullscale_AA_value = stoi_sgn(params[10]);
							shhv_.mecha_fullscale_BB_value = stoi_sgn(params[11]);
							shhv_.steer_dynamic_limit_rate = stoi_sgn(params[12]);
							shhv_.joy_angle_sub = stoi_sgn(params[13]);
							shhv_.mechanism_steering_sub = stoi_sgn(params[14]);

							shhv_arrange_.joy_operation_main = 0;//shhv_.joy_angle_main - JOY_CENTER_VOLTAGE;
							shhv_arrange_.joy_operation_sub = 0;//shhv_.joy_angle_sub - JOY_CENTER_VOLTAGE;
							shhv_arrange_.motor_voltage = (double)shhv_.motor_applied_voltage * 5.0 * 13.3 / 1024.0 / 3.3;
							shhv_arrange_.motor_current = (double)shhv_.motor_electric_current * 5.0 * 10.0 / 1024.0;
							shhv_arrange_.motor_angular_velocity = 0;//shhv_.motor_angular_velocity;
							data_type_ = DATA_TYPE_SHHV;
						}
						else if(str_code[0] == "sppm")//ペダル
						{
							sppm_.status_code = stoi_sgn(str_code[str_code.size()-1]);
							sppm_.joy_angle_main = stoi_sgn(params[1]);
							sppm_.mechanism_pedal_angle_main = stoi_sgn(params[2]);
							sppm_.motor_applied_voltage = stoi_sgn(params[3]);
							sppm_.motor_angular_velocity = stoi_sgn(params[4]);
							sppm_.motor_electric_current = stoi_sgn(params[5]);
							sppm_.pulsu_count_engine = stoi_sgn(params[6]);
							sppm_.power_supply_voltage = stoi_sgn(params[7]);
							sppm_.reserve1 = stoi_sgn(params[8]);
							sppm_.mecha_offset_adjusted_value = stoi_sgn(params[9]);
							sppm_.joy_fullscale_AA_value = stoi_sgn(params[10]);
							sppm_.mecha_fullscale_BB_brake_value = stoi_sgn(params[11]);
							sppm_.mecha_fullscale_BB_accele_value = stoi_sgn(params[12]);
							sppm_.joy_angle_sub = stoi_sgn(params[13]);
							sppm_.mechanism_angle_sub = stoi_sgn(params[14]);
							data_type_ = DATA_TYPE_SPPM;
						}
					}
				}

				read_data_.clear();
			}
		}
	}

	int getDataType() {return data_type_;}
	autoware_can_msgs::MicroBusSHHV getSHHV() {return shhv_;}
	autoware_can_msgs::MicroBusSHHVArrange getSHHVArrange() {return shhv_arrange_;}
	autoware_can_msgs::MicroBusSPPM getSPPM() {return sppm_;}
};

class CanReceiver
{
private:
	ros::NodeHandle nh_, p_nh_;
	ros::Publisher pub_shhv_, pub_shhv_arrange_, pub_sppm_;

	int device_number_;
	bool open_flag;
	ReadData read_data_;
public:
	CanReceiver(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
		, open_flag(false)
		, device_number_(-1)
	{
		std::string port;
		p_nh.getParam("port", port);
		std::cout << "port:" << port << std::endl;
		int baud = B38400;

		int fd = open(port.c_str(), O_RDONLY);     // デバイスをオープンする
		if (fd < 0) {
			std::cout << "open error" << std::endl;
			return;
		}

		struct termios tio;                 // シリアル通信設定
		tio.c_cflag += CREAD;               // 受信有効
		tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
		tio.c_cflag += CS8;                 // データビット:8bit
		tio.c_cflag += 0;                   // ストップビット:1bit
		tio.c_cflag += PARENB;              // パリティ:None

		if(cfsetispeed( &tio, baud ) == -1)
		{
			std::cout << "cfsetispeed error" << std::endl;
			return;
		}

		cfmakeraw(&tio);

		if(tcsetattr( fd, TCSANOW, &tio ) == -1)
		{
			std::cout << "ioctl" << std::endl;
			return;
		}

		if(ioctl(fd, TCSETS, &tio) == -1)
		{
			std::cout << "ioctl" << std::endl;
		}

		device_number_ = fd;

		pub_shhv_ = nh_.advertise<autoware_can_msgs::MicroBusSHHV>("/microbus/shhv", 10);
		//pub_shhv_arrange_ = nh_.advertise<autoware_can_msgs::MicroBusSHHVArrange>("/microbus/shhv_arrange", 10);
		pub_sppm_ = nh_.advertise<autoware_can_msgs::MicroBusSPPM>("/microbus/sppm", 10);

		open_flag = true;
	}

	bool is_oepn() {return open_flag;}

	void read_data()
	{
		ros::Time nowtime = ros::Time::now();

		unsigned char buf[ReadData::DATA_MAX_SIZE + 1];
		int len = read(device_number_, buf, ReadData::DATA_MAX_SIZE);
		read_data_.addData(buf, len);
		//std::cout << "type:"<< read_data_.getDataType() << std::endl;
		switch(read_data_.getDataType())
		{
		case ReadData::DATA_TYPE_SHHV:
			{
				autoware_can_msgs::MicroBusSHHV shhv = read_data_.getSHHV();
				shhv.header.stamp = nowtime;
				pub_shhv_.publish(shhv);
				/*autoware_can_msgs::MicroBusSHHVArrange shhv_arrange = read_data_.getSHHVArrange();
				shhv_arrange.header.stamp = nowtime;
				pub_shhv_arrange_.publish(shhv_arrange);*/
				break;
			}
		case ReadData::DATA_TYPE_SPPM:
			{
				autoware_can_msgs::MicroBusSPPM sppm = read_data_.getSPPM();
				sppm.header.stamp = nowtime;
				pub_sppm_.publish(sppm);
				break;
			}
		}
		//std::cout << "read size:" << len << std::endl;
		//buf[len] = '\0';
		//std::cout << buf << std::endl << std::endl;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rs232c_microbus_can_receiver");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	CanReceiver can_receiver(nh, private_nh);
	if(!can_receiver.is_oepn())
	{
		std::cout << "error : not open port" << std::endl;
		return 0;
	}

	ros::Rate rate(ros::Duration(0.01));
	while(ros::ok())
	{
		//ros::spinOnce();
		can_receiver.read_data();
		//rate.sleep();
	}
	return 0;
}
