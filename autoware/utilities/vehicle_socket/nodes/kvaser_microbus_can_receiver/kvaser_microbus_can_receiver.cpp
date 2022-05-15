#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_config_msgs/ConfigMicroBusCan.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/SteerSubInterfaceCorrection.h>
#include <std_msgs/String.h>
#include "kvaser_can.h"

std::string ID100ToString(const unsigned char data[], size_t size)
{
	std::stringstream ss;
	for(size_t i=0; i<size; i++)
	{
		ss << std::hex << std::setfill('0') << std::right << std::setw(2) << +data[i];
		if(i != size-1) ss << "_";
	}
	return ss.str();
}

class kvaser_can_receiver
{
private:
	const double STEER_SUB_INTERFACE_CORRECTION_INTARVAL_ = 0.5;//ステア補正が終了してから補正をリセットするまでの時間

	ros::NodeHandle nh_, private_nh_;
	ros::Publisher pub_microbus_can_501_, pub_microbus_can_502_, pub_microbus_can_503_, pub_tmp_;
	ros::Publisher pub_microbus_can_100_string_, pub_receiver_steer_correction_;

	ros::Subscriber sub_config_, sub_steer_plus_, sub_waypoint_param_, sub_steer_sub_interface_correction_;
	ros::Timer timer_;//車両からのステア調整が終わった場合の処理を担当

	//stroke params
	const short PEDAL_VOLTAGE_CENTER_ = 1024;//1052;//計測値は1025;
	const short PEDAL_DISPLACEMENT_CENTER_ = 1024;//計測値は1029;
	const short PEDAL_VOLTAGE_MAX_ = 161;
	const short PEDAL_DISPLACEMENT_MAX_ = 1161;
	const short PEDAL_VOLTAGE_MIN_ = 1533;
	const short PEDAL_DISPLACEMENT_MIN_ = 849;

	//liesse params
	double handle_angle_right_max = 730;
	double handle_angle_left_max = 765;
	double wheelrad_to_steering_can_value_left_ = 25009.6727514125;//liesse 20935.4958411006;//cmdのwheel指令をcanのハンドル指令に変換する係数(左回り用)
	double wheelrad_to_steering_can_value_right_ = 26765.9140133745;//liesse 20791.4464661611;//cmdのwheel指令をcanのハンドル指令に変換する係数(右回り用)
	double angle_magn_right = handle_angle_right_max / 15000;
	double angle_magn_left = handle_angle_left_max / 15000;

	KVASER_CAN kc;

	struct
	{
		bool read501, read502;
	} read_id_flag_;

	autoware_config_msgs::ConfigMicroBusCan config_;
	std::vector<short> velocity_list_;
	std::vector<short> acceleration_list_;
	std::vector<short> angle_list_;
	ros::Time prev_time_501_, prev_time_502_, prev_time_503_;
	const int list_pushback_size = 5;
	//bool steer_plus_interface_pub_;
	//int16_t steer_plus_interface_;

	autoware_msgs::WaypointParam waypoint_param_;
	int16_t steer_actual_plus_interface_;//, steer_actual_plus_param_;//インターフェース指定のステア補正値
	int16_t steer_sub_interface_correction_;//waypoint_param.steer_actual_plus_subに補正として入れるステア補正値
	ros::Time steer_sub_interface_correction_time_;//ステア補正が行われた時間

	void callbackConfig(const autoware_config_msgs::ConfigMicroBusCan::ConstPtr &msg)
	{
		config_ = *msg;
	}

	void callbackSteerPlus(const std_msgs::Int16::ConstPtr &msg)
	{
		//config_.steer_actual_plus = msg->data;
		steer_actual_plus_interface_ = msg->data;
		//pub_config_.publish(config_);
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		waypoint_param_ = *msg;
		//steer_actual_plus_param_ = msg->steer_actual_plus;
	}

	void callbackSteerSubInterfaceCorrection(const autoware_msgs::SteerSubInterfaceCorrection::ConstPtr &msg)
	{
		steer_sub_interface_correction_ = (msg->processing == true) ? msg->correct_val : 0;
		steer_sub_interface_correction_time_ = ros::Time::now();
	}

	//ステア補正が終了してから補正をリセットする
	void callbackTimer(const ros::TimerEvent& e)
	{
		if(steer_sub_interface_correction_ > 0)
		{
			steer_sub_interface_correction_ -= 10;
			if(steer_sub_interface_correction_ < 0) steer_sub_interface_correction_ = 0;
		}
		else if(steer_sub_interface_correction_ < 0)
		{
			steer_sub_interface_correction_ += 10;
			if(steer_sub_interface_correction_ > 0) steer_sub_interface_correction_ = 0;
		}

		if(steer_sub_interface_correction_ == 0)
			timer_.stop();
	}
public:
	kvaser_can_receiver(ros::NodeHandle nh, ros::NodeHandle p_nh, int kvaser_channel)
		: nh_(nh)
		, private_nh_(p_nh)
		, steer_sub_interface_correction_(0)
		, steer_sub_interface_correction_time_(ros::Time(0))
	{
		prev_time_502_ = ros::Time::now();
		read_id_flag_.read501 = read_id_flag_.read502 = false;
		kc.init(kvaser_channel, canBITRATE_500K);

		nh_.param<double>("/vehicle_info/wheelrad_to_steering_can_value_left", wheelrad_to_steering_can_value_left_, 25009.6727514125);
		nh_.param<double>("/vehicle_info/wheelrad_to_steering_can_value_right", wheelrad_to_steering_can_value_right_, 26765.9140133745);

		pub_microbus_can_100_string_ = nh_.advertise<std_msgs::String>("/microbus/can_receive100_string", 10);
		pub_microbus_can_501_ = nh_.advertise<autoware_can_msgs::MicroBusCan501>("/microbus/can_receive501", 10);
		pub_microbus_can_502_ = nh_.advertise<autoware_can_msgs::MicroBusCan502>("/microbus/can_receive502", 10);
		pub_microbus_can_503_ = nh_.advertise<autoware_can_msgs::MicroBusCan503>("/microbus/can_receive503", 10);
		pub_receiver_steer_correction_ = nh_.advertise<std_msgs::Int16>("/microbus/receiver_steer_correction", 10);
		pub_tmp_ = nh_.advertise<std_msgs::String>("/microbus/receiver_tmp", 10);

		sub_config_ = nh_.subscribe("/config/microbus_can", 10, &kvaser_can_receiver::callbackConfig, this);//このノードのconfig設定
		sub_steer_plus_ = nh_.subscribe("/microbus/steer_plus", 10, &kvaser_can_receiver::callbackSteerPlus, this);//steer補正
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 10, &kvaser_can_receiver::callbackWaypointParam, this);
		sub_steer_sub_interface_correction_ = nh_.subscribe("/microbus/steer_sub_interface_correction", 10, &kvaser_can_receiver::callbackSteerSubInterfaceCorrection, this);
		timer_ = nh_.createTimer(ros::Duration(0.1), &kvaser_can_receiver::callbackTimer, this);
		timer_.stop();
	}

	bool isOpen() {return kc.isOpen();}

	void read_wait()
	{
		ros::Time nowtime = ros::Time::now();
		canStatus res = kc.read_wait(100);
		//if(kc.get_id() == 0x501 || kc.get_id() == 0x502 || kc.get_id() == 0x503) kc.printReader();
		//if(kc.get_id() == 0x100) kc.printReader();
		kc.printReader();

		if(res == canStatus::canOK)
		{
			switch(kc.get_id())
			{
			case 0x100:
				{
					unsigned char data[KVASER_CAN::READ_DATA_SIZE];
					kc.get_read_data(data);
					std_msgs::String str;
					str.data = ID100ToString(data, KVASER_CAN::READ_DATA_SIZE);
					pub_microbus_can_100_string_.publish(str);
					break;
				}
			case 0x501:
			    {
				    unsigned char data[KVASER_CAN::READ_DATA_SIZE];
					kc.get_read_data(data);
					autoware_can_msgs::MicroBusCan501 can;
					can.header.stamp = ros::Time::now();

					can.emergency = (data[0] == 0x55);
					unsigned char dmode0 = data[0] & 0x0F;
					unsigned char dmode1 = data[1] & 0x0F;
					//can.drive_auto = (dmode1==0x10 || dmode1==0x11) ? 0xA : 0;
					/*switch(dmode1)
					{
					case 0x0A:
						can.drive_auto = true;
						break;
					case 0x0B:
						can.drive_auto = true;
						break;
					default:
						can.drive_auto = false;
					}*/
					switch(dmode1)
					{
					case autoware_can_msgs::MicroBusCan501::STEER_AUTO:
						can.drive_auto = autoware_can_msgs::MicroBusCan501::STEER_AUTO;
						can.drive_mode = autoware_can_msgs::MicroBusCan501::DRIVE_MODE_STROKE;
						break;
					case autoware_can_msgs::MicroBusCan501::STEER_AUTO+1:
						can.drive_auto = autoware_can_msgs::MicroBusCan501::STEER_AUTO;
						can.drive_mode = autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY;
						break;
					default:
						can.drive_auto = 0;
						can.drive_mode = 0;
						break;
					}
					//can.drive_auto = (dmode == 0x0A);
					unsigned char smode = data[1] & 0xF0;
					can.steer_auto = smode >> 4;

					unsigned char *vel_tmp = (unsigned char*)&can.stroke_reply;
					vel_tmp[0] = data[5];  vel_tmp[1] = data[4];

					unsigned char *str_tmp = (unsigned char*)&can.steering_angle_reply;
					str_tmp[0] = data[3];  str_tmp[1] = data[2];

					unsigned char *stroke_tmp = (unsigned char*)&can.pedal;
					//stroke_tmp[?] = data[?];  stroke_tmp[?] = data[?];

					can.read_counter = kc.get_read_counter();

					if(data[6] & 0x80) can.emergency_stop = 2;
					else if(data[6] & 0x40) can.emergency_stop = 1;
					else can.emergency_stop = 0;
					/*if(data[6] & 0x20 != 0) can.side_brake = 2;
					else if(data[6] & 0x10 != 0) can.side_brake = 1;
					else can.side_brake = 0;
					if(data[6] & 0x08 != 0) can.automatic_door = 2;
					else if(data[6] & 0x04 != 0) can.automatic_door = 1;
					else can.automatic_door = 0;*/
					if(data[6] & 0x04) can.accel_intervention = 1;
					else can.accel_intervention = 0;
					can.blinker = (data[6] & 0x02) ? true : false;
					can.auto_log = (data[7] & 0x20) ? true : false;
					/*if(data[6] & 0x80 != 0) can.emergency_stop = 2;
					else if(data[6] & 0x40 != 0) can.emergency_stop = 1;
					else can.emergency_stop = 0;
					can.engine_start = (data[6] & 0x20 != 0) ? true : false;
					can.ignition = (data[6] & 0x10 != 0) ? true : false;
					can.wiper = (data[6] & 0x08 != 0) ? true : false;
					can.light_high = (data[6] & 0x04 != 0) ? true : false;
					can.light_low = (data[6] & 0x02 != 0) ? true : false;
					can.light_small = (data[6] & 0x01 != 0) ? true : false;
					can.horn = (data[7] & 0x80 != 0) ? true : false;
					can.hazard = (data[7] & 0x40 != 0) ? true : false;
					can.blinker_right = (data[7] & 0x20 != 0) ? true : false;
					can.blinker_left = (data[7] & 0x10 != 0) ? true : false;
					can.shift = data[7] & 0x0F;*/

					pub_microbus_can_501_.publish(can);
					read_id_flag_.read501 = true;
					prev_time_501_ = nowtime;

					std::stringstream ss;
					ss << std::hex << (int)(data[7] & 0x20);
					pub_tmp_.publish(ss.str());
					break;
			    }
			case 0x502:
			    {
				    unsigned char data[KVASER_CAN::READ_DATA_SIZE];
					kc.get_read_data(data);
					autoware_can_msgs::MicroBusCan502 can;
					can.header.stamp = nowtime;

					unsigned char *vel_tmp = (unsigned char*)&can.velocity_actual;
					vel_tmp[0] = data[7];  vel_tmp[1] = data[6];
					can.velocity_actual /= 2.0;//割る２は2022_05_15で入れた暫定対処

					short acceleration = 0;
					can.cycle_time = nowtime.toSec() - prev_time_502_.toSec();
					if(velocity_list_.size() != 0)
					{
						short acc = (velocity_list_[0] - can.velocity_actual) / can.cycle_time;
						can.acceleration_actual = acc;
						acceleration_list_.insert(acceleration_list_.begin(), acc);
						if(acceleration_list_.size() > list_pushback_size) acceleration_list_.resize(list_pushback_size);
						int sum = 0;
						for(int i=0; i<acceleration_list_.size(); i++) sum += acceleration_list_[i];
						can.acceleration_average = sum / acceleration_list_.size();

						std::vector<short> acceleration_med(acceleration_list_.size());
						std::copy(acceleration_list_.begin(), acceleration_list_.end(), acceleration_med.begin());
						std::sort(acceleration_med.begin(), acceleration_med.end());
						can.acceleration_median = acceleration_med[(int)std::round(acceleration_med.size()/(double)2.0)];
					}


					velocity_list_.insert(velocity_list_.begin(), can.velocity_actual);
					if(velocity_list_.size() > list_pushback_size) velocity_list_.resize(list_pushback_size);
					int sum = 0;
					for(int i=0; i<velocity_list_.size(); i++) sum += velocity_list_[i];
					can.velocity_average = sum / velocity_list_.size();

					std::vector<short> velocity_med(velocity_list_.size());
					std::copy(velocity_list_.begin(), velocity_list_.end(), velocity_med.begin());
					std::sort(velocity_med.begin(), velocity_med.end());
					can.velocity_median = velocity_med[(int)std::round(velocity_med.size()/(double)2.0)];

					can.velocity_mps = (double)can.velocity_actual / 100.0 / 3.6;
					unsigned char *str_tmp = (unsigned char*)&can.angle_actual;
					str_tmp[0] = data[5];  str_tmp[1] = data[4];
					can.angle_actual += waypoint_param_.steer_actual_plus + waypoint_param_.steer_actual_plus_sub + steer_sub_interface_correction_;//steer_actual_plus_param_;//config_.steer_actual_plus;//(330);//(config_.angle_actual_correction);//ズレ補正
					std_msgs::Int16 correction_msg;
					correction_msg.data = waypoint_param_.steer_actual_plus_sub + steer_sub_interface_correction_;
					pub_receiver_steer_correction_.publish(correction_msg);
					std::cout << waypoint_param_.steer_actual_plus << "," << waypoint_param_.steer_actual_plus_sub << std::endl;
					angle_list_.insert(angle_list_.begin(), can.angle_actual);
					if(angle_list_.size() == list_pushback_size) angle_list_.resize(list_pushback_size);
					if(can.velocity_actual >= 0) can.angle_deg = can.angle_actual * angle_magn_left;
					else can.angle_deg = can.angle_actual * angle_magn_right;
					//if(can.angle_actual > 0) can.wheel_deg = can.angle_actual / wheelrad_to_steering_can_value_left_;
					//else can.wheel_deg = can.angle_actual / wheelrad_to_steering_can_value_right_;

					unsigned char *steer_voltage = (unsigned char*)&can.angle_target_voltage;
					steer_voltage[0] = data[3];  steer_voltage[1] = data[2];

					unsigned char status0 = data[0],  status1 = data[1];
					can.clutch = (status1 & 0x40) ? false : true;

					can.read_counter = kc.get_read_counter();

					pub_microbus_can_502_.publish(can);
				    read_id_flag_.read502 = true;
					ros::Duration time_diff = nowtime - prev_time_502_;
					prev_time_502_ = nowtime;
					break;
			    }
			case 0x503:
			    {
				    unsigned char data[KVASER_CAN::READ_DATA_SIZE];
					kc.get_read_data(data);
					autoware_can_msgs::MicroBusCan503 can;
					can.header.stamp = ros::Time::now();

					unsigned char *voltage_tmp = (unsigned char*)&can.pedal_voltage;//23 ペダル指令電圧データ
					voltage_tmp[0] = data[3];  voltage_tmp[1] = data[2];

					unsigned char *displacement_tmp = (unsigned char*)&can.pedal_can_displacement;//45 ペダル変位量
					displacement_tmp[0] = data[5];  displacement_tmp[1] = data[4];

					unsigned char *engine_rotation_tmp = (unsigned char*)&can.engine_rotation;//67 エンジン回転数
					engine_rotation_tmp[0] = data[7];  engine_rotation_tmp[1] = data[6];

					unsigned char status0 = data[0],  status1 = data[1];
					can.clutch = (status1 & 0x40) ? false : true;

					can.read_counter = kc.get_read_counter();

					can.pedal_voltage_displacement = PEDAL_VOLTAGE_CENTER_ - can.pedal_voltage;
					can.pedal_displacement = can.pedal_voltage_displacement;

					pub_microbus_can_503_.publish(can);
					std::cout << "pedal_voltage : " << can.pedal_voltage << std::endl;
					std::cout << "pedal_displacement : " << can.pedal_displacement << std::endl;
					std::cout << "engine_rotation : " << can.engine_rotation << std::endl;
				    read_id_flag_.read501 = read_id_flag_.read502 = false;
					ros::Duration time_diff = nowtime - prev_time_503_;
					prev_time_503_ = nowtime;
					break;
			    }
			}
		}

		ros::Duration ros_time_diff = nowtime - steer_sub_interface_correction_time_;
		double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
		if(time_diff > STEER_SUB_INTERFACE_CORRECTION_INTARVAL_)
			timer_.start();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_prius_can_receiver");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	int kvaser_channel;
	private_nh.param("kvaser_channel", kvaser_channel, 0);
	kvaser_can_receiver kcr(nh, private_nh, kvaser_channel);
	if(kcr.isOpen() == false)
	{
		std::cerr << "error : open" << std::endl;
	}

	while(ros::ok())
	{
		kcr.read_wait();
		ros::spinOnce();
	}

	return 0;
}