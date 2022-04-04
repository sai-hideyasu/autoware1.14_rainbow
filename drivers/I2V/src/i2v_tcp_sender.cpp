/**
 * @file i2v_tcp_sender.cpp
 * @brief 路車間通信アプリケーション規格での信号情報を送信するクラスのソースフィアル
 * @author Hideyasu Sai(Saitama Institute of Technology)
 * @date 2021/12/15
 * @details 参考サイト：https://boostjp.github.io/tips/network/tcp.html
 */
#include "i2v_tcp_sender.h"
//#include "i2v_tcp_reader.h"
#include "other.h"
#include <fstream>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/TrafficLightI2V.h>
const int TRAFFIC_LIGHT_RED = 0;
const int TRAFFIC_LIGHT_GREEN = 1;
const int TRAFFIC_LIGHT_YELLOW = 10;
const int TRAFFIC_LIGHT_YELLOW_RED = 11;
const int TRAFFIC_LIGHT_ARROW = 12;
const int TRAFFIC_LIGHT_UNKNOWN = 2;

/**
 * @fn struct tm getSystemTime(suseconds_t &nanosec)
 * @brief 時間をOSから取得
 * @param[out] nanosec 取得したミリ秒
 * @return ミリ秒以外の時間情報となるtm構造体
 * @details
*/
struct tm getSystemTime(suseconds_t &nanosec)
{
	struct timeval get_time_ret;
	gettimeofday(&get_time_ret, NULL);
	struct tm* time_struct = localtime(&get_time_ret.tv_sec);
	nanosec = get_time_ret.tv_usec;
	return *time_struct;
}

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

/**
 * @fn I2V_Sender::I2V_Sender(const ros::NodeHandle nh, ros::NodeHandle pnh)
 * @brief I2V_Senderのコンストラクタ
 * @param[in] nh ROSノードハンドル(global)
 * @param[in] pnh ROSノードハンドル(private)
 * @return なし
 * @details rosrun時に\n
 *  guid\n
 *  ip\n
 *  port\n
 *  aes128_key\n
 *  aes128_constract_vec\n
 *  use_time_info\n
 *  const_latitude_deg\n
 *  const_lontitude_deg\n
 *  const_velocty_kh\n
 * を読み込む\n
 * ソケット通信接続ごに各種ROSトピックのsubscribe設定を行う
*/
I2V_Sender::I2V_Sender(const ros::NodeHandle nh, const ros::NodeHandle pnh)
	: nh_(nh)
	, pnh_(pnh)
	//, io_()
	//, socket_(io_)
	, timer_count_(0)
	, signal_id_only_(false)
	, read_gnss_direction_flag_(false)
	, read_gnss_time_flag_(false)
	, read_velocity_flag_(false)
	, exchange_flag_(false)
	, first_write_(false)
	, prev_light_color_(TRAFFIC_LIGHT_UNKNOWN)
	, prev_orig_color_(0x0)
	, signal_stop_line_time_(-1)
{
	guid_ = pnh_.param<int>("guid", 5963);
	ip_ = pnh_.param<std::string>("ip", "210.156.173.53");
	port_ = pnh_.param<int>("port", 56974);
	aes128_key_ = string2vector(pnh_.param<std::string>("aes128_key", "0000000000000000"));
	aes128_constract_vec_ = string2vector(pnh_.param<std::string>("aes128_constract_vec", "0000000000000000"));
	use_time_info_ = pnh_.param<int>("use_time_info", 0.0);
	const_latitude_deg_ = pnh_.param<double>("const_latitude_deg", 0.0);
	const_longitude_deg_ = pnh_.param<double>("const_longitude_deg", 0.0);
	const_velocity_kh_ = pnh_.param<double>("const_velocity_kh", 0.0);
	signalID_ = pnh_.param<int>("signal_id", 0);//std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaa," << signalID_ << std::endl;
	/*std::string signal_id_str = signal_id_str = pnh_.param<std::string>("signal_id", "0");std::cout << "aaaaaaaaaaaaa," << signal_id_str << std::endl;
	signalID_ = std::stoi(signal_id_str, nullptr, 16);*/
	infomationID_ = pnh_.param<int>("infomation_id", 0);
	use_load_id_ = pnh_.param<int>("use_load_id", 0);
	use_car_signal_id_ = pnh_.param<int>("use_car_signal_id", 0);
	use_arrow_ = pnh_.param<int>("use_arrow", 0);
	std::string light_color_str = pnh_.param<std::string>("light_color", "light_color");

	printInit();

	// connect server
	/*boost::system::error_code error;
	//boost::asio::ip::tcp::endpoint endpoint = boost::asio::ip::tcp::endpoint{boost::asio::ip::address::from_string(ip_), port_};
	boost::asio::ip::udp::endpoint endpoint = boost::asio::ip::udp::endpoint{boost::asio::ip::udp::v4(), port_};
	socket_.connect(endpoint, error);
	if (error) {
		std::cout << "接続失敗(送信) : " << error.message() << std::endl;
	}
	else {
		std::cout << "接続成功(送信)" << std::endl;
	}*/
 	socket_ = socket(AF_INET, SOCK_DGRAM, 0);
	addr_.sin_family = AF_INET;
	addr_.sin_addr.s_addr = inet_addr(ip_.c_str());
	addr_.sin_port = htons(port_); 
	if(socket_ == -1){
		std::cout << "接続失敗(送信) : " << std::endl; socket_ok_ = false;
	}
	else {
		std::cout << "接続成功(送信)" << std::endl; socket_ok_ = true;
	}

	sub_config_ = nh_.subscribe("/config/i2v", 1, &I2V_Sender::callbackConfig, this);
	sub_gnss_direction_ = nh_.subscribe("/gnss_direction", 1, &I2V_Sender::callbackGnssDirection, this);
	sub_gnss_time_ = nh_.subscribe("/gnss_time", 1, &I2V_Sender::callbackGnssTime, this);
	sub_gnss_surface_speed_ = nh_.subscribe("/gnss_surface_speed", 1, &I2V_Sender::callbackGnssSurfaceSpeed, this);
	sub_exchange_flag_ = nh_.subscribe("/i2v/exchange", 1, &I2V_Sender::callbackExchangeFlag, this);
	sub_signal_id_only_ = nh_.subscribe("/i2v/signal_id_only", 1, &I2V_Sender::callbackSignalIdOnly, this);
	sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 1, &I2V_Sender::callbackWaypointParam, this);
	sub_waypoints_ = nh_.subscribe("/final_waypoints", 1, &I2V_Sender::callbackWaypoints, this);
	sub_decode_data_ = nh_.subscribe("i2v_decode_data", 1, &I2V_Sender::callbackDecodeData, this);

	pub_signal_stat_ = nh_.advertise<autoware_msgs::TrafficLight>(light_color_str, 10, true);
	pub_signal_stat_i2v_ = nh_.advertise<autoware_msgs::TrafficLightI2V>(light_color_str + "_i2v", 10, true);
	//pub_decode_data_ = nh_.advertise<std_msgs::UInt8MultiArray>("i2v_decode_data", 10, false);

	timer_ = nh_.createTimer(ros::Duration(1.0/SEND_HZ), &I2V_Sender::callbackTimer, this);

	config_.red_time_sec = 5;
}

/**
 * @fn void I2V_Sender::printInit() const
 * @brief I2V_Senderの初期設定をコンソールに表示
 * @return なし
 * @details
*/
void I2V_Sender::printInit() const
{
	std::cout << "GUID," << guid_ << std::endl;
	std::cout << "IP ADDRESS," << ip_ << std::endl;
	std::cout << "PORT_," << port_ << std::endl;
	std::cout << "AES128_KEY," << vector2string(aes128_key_) << std::endl;
	std::cout << "AES128_CONSTRACT_VEC," << vector2string(aes128_constract_vec_) << std::endl;
	std::cout << "USE_LOAD_ID," << +use_load_id_ << std::endl;
	std::cout << "USE_TIME_INFO," << use_time_info_ << std::endl;
	std::cout << "SIGNAL_ID,," << std::hex << signalID_ <<  std::dec << std::endl;
	std::cout << "INFOMATION_ID,," << std::hex << infomationID_ <<  std::dec << std::endl;
	std::cout << "CONST_LATITUDE_DEG," << const_latitude_deg_ << std::endl;
	std::cout << "CONST_LONGITUDE_DEG," << const_longitude_deg_ << std::endl;
	std::cout << "CONST_VELOCITY_KH," << const_velocity_kh_ << std::endl;
	std::cout << "RIGHT_COLOR," << pub_signal_stat_.getTopic() << std::endl;
	std::cout << std::endl;
}

/**
 * @fn bool I2V_Sender::execJudg() const
 * @brief 信号情報送信可能判定をする
 * @return 送信可能ならTRUE
 * @details 時間、緯度経度、速度を判定する
*/
bool I2V_Sender::execJudg() const
{
	bool time_flag;
	switch (use_time_info_)
	{
	case USE_TIME_SYSTEM:
		time_flag = true;
		break;
	case USE_TIME_GNSS:
		time_flag = read_gnss_time_flag_;
		break;
	default:
		time_flag = false;
		break;
	}

	bool direction_flag;
	if(signal_id_only_ == true) direction_flag = true;
	else if(const_latitude_deg_ != 0.0 || const_longitude_deg_ != 0.0) direction_flag = true;
	else direction_flag = read_gnss_direction_flag_;

	bool velocity_flag;
	if(const_velocity_kh_ >= 0) velocity_flag = true;
	else velocity_flag = read_velocity_flag_;
	std::cout << std::boolalpha << time_flag << "," << direction_flag << "," << velocity_flag << std::endl;
	return (time_flag && direction_flag && velocity_flag) ? true : false;
}

void I2V_Sender::callbackConfig(const autoware_config_msgs::ConfigI2V::ConstPtr &msg)
{
	config_ = *msg;
}

/**
 * @fn I2V_Sender::callbackGnssDirection(const autoware_msgs::GnssDirection::ConstPtr &msg)
 * @brief 緯度経度を取得するsubscribe
 * @param[in] msg autoware_msgs::GnssDirection
 * @return ミリ秒以外の時間情報となるtm構造体
 * @details
*/
void I2V_Sender::callbackGnssDirection(const autoware_msgs::GnssDirection::ConstPtr &msg)
{
	gnss_direction_ = *msg;
	read_gnss_direction_flag_ = true;
}

void I2V_Sender::callbackGnssTime(const autoware_system_msgs::Date::ConstPtr &msg)
{
	gnss_date_ = *msg;
	read_gnss_time_flag_ = true;

	std::stringstream ss;
	ss << std::setfill('0') << std::setw(4) << (uint16_t)gnss_date_.year << "/" << std::setw(2)
		<< +(uint8_t)gnss_date_.month << "/"
		<< +(uint8_t)gnss_date_.day << "-"
		<< +(uint8_t)gnss_date_.hour+9 << ":"
		<< +(uint8_t)gnss_date_.min << ":"
		<< gnss_date_.sec;
	time_string_ = ss.str();
}

void I2V_Sender::callbackGnssSurfaceSpeed(const autoware_msgs::GnssSurfaceSpeed::ConstPtr &msg)
{
	velocity_ms_ = msg->surface_speed;
	read_velocity_flag_ = true;
}

void I2V_Sender::callbackExchangeFlag(const std_msgs::Bool::ConstPtr &msg)
{
	exchange_flag_ = true;
}

void I2V_Sender::callbackSignalIdOnly(const std_msgs::Bool::ConstPtr &msg)
{
	signal_id_only_ = msg->data;
}

void I2V_Sender::publishLightColor(const int light_color, const int orig_light, const uint16_t change_time,
	const std::string receive_date, const std::string send_date)
{
	//std::cout << "publish," << light_color << "," << prev_light_color_ << std::endl;

	ros::Time nowtime = ros::Time::now();

	autoware_msgs::TrafficLightI2V light_msg_i2v;
	light_msg_i2v.header.stamp = nowtime;
	light_msg_i2v.traffic_light = light_color;
	light_msg_i2v.receive_date = receive_date;
	light_msg_i2v.send_date = send_date;
	light_msg_i2v.current_date = time_string_;
	pub_signal_stat_i2v_.publish(light_msg_i2v);

	if(light_color != prev_light_color_ || orig_light != prev_orig_color_ || change_time != prev_change_time_)
	{
		autoware_msgs::TrafficLight light_msg;
		light_msg.header.stamp = nowtime;
		light_msg.traffic_light = light_color;
		//light_msg.receive_date = receive_date;
		//light_msg.send_date = send_date;
		light_msg.device_type = autoware_msgs::TrafficLight::DEVICE_I2V;
		light_msg.i2v_orig_light = orig_light;
		light_msg.i2v_change_time = change_time;
		pub_signal_stat_.publish(light_msg);

		prev_light_color_ = light_color;
		prev_orig_color_ = orig_light;
		prev_change_time_ = change_time;
	}
}

void I2V_Sender::callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
{
	//exchange_flag_ = (msg->i2v_start == 0) ? false : true;
	//exchange_flag_ = (msg->i2v_start == light_color_id_) ? true : false;
	//exchange_flag_ = (msg->signal_select_topic == pub_signal_stat_.getTopic()) ? true : false;
	//std::cout << msg->signal_select_topic << "," << pub_signal_stat_.getTopic() << std::endl;
	std::string param_topic = msg->signal_select_topic;
	std::stringstream param_ss;
	if(param_topic[0] == '/')
		param_ss << param_topic;
	else
		param_ss << "/" << param_topic;
	std::cout << param_ss.str() << "," << pub_signal_stat_.getTopic() << "," << std::boolalpha << exchange_flag_<< std::endl;
	if(param_ss.str() == pub_signal_stat_.getTopic())
	{
		exchange_flag_ = true;
	}
	else
	{
		exchange_flag_ = false;
		//publishLightColor(TRAFFIC_LIGHT_UNKNOWN);
	}
}

double nasukakuRad(const double base_ang_rad, const double next_ang_rad)
{
	tf::Quaternion current_qua = tf::createQuaternionFromYaw(base_ang_rad);
	tf::Quaternion way_qua = tf::createQuaternionFromYaw(next_ang_rad);
	tf::Quaternion sa_qua = way_qua * current_qua.inverse();
	double sa_roll, sa_pitch, sa_yaw;
	tf::Matrix3x3(way_qua).getRPY(sa_roll, sa_pitch, sa_yaw);
	return sa_yaw;
}

double mathDistance(const geometry_msgs::Point &p1, geometry_msgs::Point &p2)
{
	double x = p1.x - p2.x;
	double y = p1.y - p2.y;
	double z = p1.z - p2.z;
	return sqrt(x*x + y+y + z*z);
}

double euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
	double x1 = p1.x,  x2 = p2.x;
	double y1 = p1.y,  y2 = p2.y;
	double z1 = p1.z,  z2 = p2.z;
	double xd = x1 - x2,  yd = y1 - y2,  zd = z1 - z2;
	return std::sqrt(xd*xd + yd*yd + zd*zd);
}

//現在の位置から停止線までの時間を経路情報から作成
double signalStopLineTime(const autoware_msgs::Lane &lane)
{
	double s_sum = 0;
	bool flag = false;
	for(int waycou=1; waycou<lane.waypoints.size(); waycou++)
	{
		const geometry_msgs::Point &po      = lane.waypoints[waycou].pose.pose.position;
		const geometry_msgs::Point &po_prev = lane.waypoints[waycou-1].pose.pose.position;
		double vel = lane.waypoints[waycou-1].twist.twist.linear.x;
		double distance = euclideanDistance(po, po_prev);
		s_sum += distance / vel;

		if(lane.waypoints[waycou].waypoint_param.signal_stop_line > 0)
		{
			flag = true;
			break;
		}
	}

	if(flag == true) return s_sum;
	else return -1;
}

void I2V_Sender::callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
{
	future_data_.clear();

	const std::vector<autoware_msgs::Waypoint> &waypoints = msg->waypoints;
	size_t max_cou = (FUTURE_COUNT_MAX<waypoints.size()) ? FUTURE_COUNT_MAX : waypoints.size();
	for(size_t way_cou=1; way_cou<max_cou; way_cou++)
	{
		const autoware_msgs::WaypointParam &param = waypoints[way_cou].waypoint_param;
		{
			geometry_msgs::Point p_prev = waypoints[way_cou-1].pose.pose.position;
			geometry_msgs::Point p_orig = waypoints[way_cou].pose.pose.position;
			double distance = mathDistance(p_prev, p_orig);
			
		}
		{
			double current_lat_rad = gnss_direction_.fix_latitude_rad;
			double way_lat_rad = (param.gnss_latitude / 100.0) * M_PI / 180.0;
			double sa_yaw_deg = nasukakuRad(current_lat_rad, way_lat_rad) * 180.0 / M_PI;
			int16_t val = sa_yaw_deg * (1/FUTURE_OFFSET_LATITUDE_COEF);
		}
		{
			double current_lon_rad = gnss_direction_.fix_longitude_rad;
			double way_lon_rad = (param.gnss_longitude / 100.0) * M_PI / 180.0;
			double sa_yaw_deg = nasukakuRad(current_lon_rad, way_lon_rad) * 180.0 / M_PI;
			int16_t val = sa_yaw_deg * (1/FUTURE_OFFSET_LATITUDE_COEF);
		}
		{
			double current_azi_rad = gnss_direction_.azimuth_rad;
			double way_azi_rad = (param.gnss_azimuth / 100.0) * M_PI / 180.0;
			double sa_yaw_deg = nasukakuRad(current_azi_rad, way_azi_rad) * 180.0 / M_PI;
			int16_t val = sa_yaw_deg * (1/FUTURE_OFFSET_LATITUDE_COEF);
		}
	}


	//現在の位置から停止線までの時間を経路情報から作成
	signal_stop_line_time_ = signalStopLineTime(*msg);
}

void I2V_Sender::send_handler(const boost::system::error_code& error, std::size_t len)
{
		// do nothing
}

int I2V_Sender::singleCircleColor(const CarSignalInfo2 &info)
{
	uint8_t circle = info.getCircleColorDisplay();
	uint8_t arrow = info.getArrowColorDisplay();
	std::cout << "cir," << +circle << "," << +arrow << std::endl;
	int light_color = TRAFFIC_LIGHT_UNKNOWN;
	if(circle == 0x1)//青
	{
		light_color = TRAFFIC_LIGHT_GREEN;
	}
	else if(circle == 0x2)//黃
	{
		light_color = TRAFFIC_LIGHT_YELLOW;
	}
	else if(arrow == use_arrow_ && use_arrow_ != 0)
	{
		light_color = TRAFFIC_LIGHT_ARROW;
	}
	else if(circle == 0x3)
	{
		light_color = TRAFFIC_LIGHT_RED;
	}
	return light_color;
}

void I2V_Sender::light_color_publish(const i2v::READER &reader)
{
	if(reader.getServeceInfoSize() >= use_load_id_ && use_load_id_ != 0)
	{
		ServiceLoadSignalInfo service_info = reader.getServiceInfo(use_load_id_-1);//指定した方路ID情報を取得
		if(service_info.getCarSignalInfoSize() >= use_car_signal_id_ && use_car_signal_id_!=0)
		{
			CarSignalInfo1 car_signal_info1 = service_info.getCarSignalInfo(use_car_signal_id_-1);//指定した信号ID情報を取得
			if(car_signal_info1.getInfo2Size() > 0)
			{
				//std::cout << "aaa," << +car_signal_info1.getInfo2(0).getCircleColorDisplay() << std::endl;
				if(signal_stop_line_time_ < 0)//停止線情報までの時間がない
				{
					//if(exchange_flag_ == true)
					{
						CarSignalInfo2 car_signal_info2 = car_signal_info1.getInfo2(0);
						int light_color = singleCircleColor(car_signal_info2);
						publishLightColor(light_color, light_color, car_signal_info2.getMaxRemainingSec10m(), reader.recieveDateStrig(), reader.sendDateString());
						return;
					}
				}
				else//停止線情報までの時間がある
				{
					size_t info_list_size = car_signal_info1.getOutputChangeCount();
					double time_sum = 0;
					int first_color = TRAFFIC_LIGHT_UNKNOWN;
					uint16_t change_time = 0;
					bool red_flag = false;
					for(size_t list_cou=0; list_cou<info_list_size; list_cou++)
					{
						CarSignalInfo2 car_signal_info2 = car_signal_info1.getInfo2(list_cou);

						int color = singleCircleColor(car_signal_info2);
						if(list_cou == 0)
						{
							first_color = color;
							change_time = car_signal_info2.getMaxRemainingSec10m();
						}

						/*if(color == TRAFFIC_LIGHT_RED && list_cou == 0)//最初の信号情報が赤だったらそのまま赤を返す
						{
							publishLightColor(TRAFFIC_LIGHT_RED, reader.recieveDateStrig(), reader.sendDateString());
							return;
						}
						else if(color == TRAFFIC_LIGHT_GREEN && list_cou == 0)
						{
							first_green_flag = true;
						}*/

						if(color == TRAFFIC_LIGHT_RED)
						{
							red_flag = true;
							break;
						}
						else
						{
							uint16_t time = car_signal_info2.getMinRemainingSec10m();
							time_sum += time / (double)10.0;
						}
					}

					if(first_color == TRAFFIC_LIGHT_RED)
					{
						publishLightColor(TRAFFIC_LIGHT_RED, first_color, change_time, reader.recieveDateStrig(), reader.sendDateString());
					}
					else if(first_color == TRAFFIC_LIGHT_GREEN)
					{
						publishLightColor(TRAFFIC_LIGHT_GREEN, first_color, change_time, reader.recieveDateStrig(), reader.sendDateString());
					}
					else if(first_color == TRAFFIC_LIGHT_YELLOW)
					{
						if(red_flag == true && time_sum < config_.red_time_sec) //赤信号までの時間がconfig_.red_time_sec以下の場合は赤信号とする
						{
							publishLightColor(TRAFFIC_LIGHT_YELLOW_RED, first_color, change_time, reader.recieveDateStrig(), reader.sendDateString());
						}
						else
						{
							publishLightColor(TRAFFIC_LIGHT_YELLOW, first_color, change_time, reader.recieveDateStrig(), reader.sendDateString());
						}
					}

					return;
				}
			}
		}
	}

	publishLightColor(TRAFFIC_LIGHT_UNKNOWN, TRAFFIC_LIGHT_UNKNOWN, 0, reader.recieveDateStrig(), reader.sendDateString());
}

void I2V_Sender::callbackTimer(const ros::TimerEvent& e)
{
	if(!execJudg())
	{
		std::cout << "まだ送信条件を満たしていません" << std::endl;
		return;
	}

	{
		std::cout << "counter: " << timer_count_ << std::endl;
		i2v::SENDER sender;
		sender.clear();
		sender.setCounter(timer_count_);
		sender.setGUID(guid_, 0);
		sender.setSignalID(signalID_);
		sender.setInfomationID(infomationID_);

		/*switch(use_time_info_)
		{
		case USE_TIME_SYSTEM:
			suseconds_t nanosec;
			struct tm system_time= getSystemTime(nanosec);
			sender.setYear((uint16_t)system_time.tm_year+1900);
			sender.setMon((uint8_t)system_time.tm_mon+1);
			sender.setDay((uint8_t)system_time.tm_mday);
			sender.setHour((uint8_t)system_time.tm_hour);
			sender.setMin((uint8_t)system_time.tm_min);
			sender.setNSec((uint16_t)system_time.tm_sec*1000 + nanosec/1000);
			break;
		case USE_TIME_GNSS:
			sender.setYear((uint16_t)gnss_date_.year);
			sender.setMon((uint8_t)gnss_date_.month);
			sender.setDay((uint8_t)gnss_date_.day);
			sender.setHour((uint8_t)gnss_date_.hour);
			sender.setMin((uint8_t)gnss_date_.min);
			sender.setNSec((uint16_t)gnss_date_.sec*1000);
			break;
		}*/
		if(use_time_info_ == USE_TIME_SYSTEM)
		{
			suseconds_t nanosec;
			struct tm system_time= getSystemTime(nanosec);
			sender.setYear((uint16_t)system_time.tm_year+1900);
			sender.setMon((uint8_t)system_time.tm_mon+1);
			sender.setDay((uint8_t)system_time.tm_mday);
			sender.setHour((uint8_t)system_time.tm_hour);
			sender.setMin((uint8_t)system_time.tm_min);
			sender.setNSec((uint16_t)system_time.tm_sec*1000 + nanosec/1000);
		}
		else if(use_time_info_ == USE_TIME_GNSS)
		{
			sender.setYear((uint16_t)gnss_date_.year);
			sender.setMon((uint8_t)gnss_date_.month);
			sender.setDay((uint8_t)gnss_date_.day);
			sender.setHour((uint8_t)gnss_date_.hour+9);
			sender.setMin((uint8_t)gnss_date_.min);
			sender.setNSec((uint16_t)(gnss_date_.sec*1000));
		}
		sender.setExchangeFlag((exchange_flag_ == true) ? 1 : 0);

		double vel = (const_velocity_kh_ >= 0) ? const_velocity_kh_ / 3.6 : velocity_ms_;
		if(signal_id_only_ == true)
		{
			sender.setLatitudeDeg(0);
			sender.setLongitudeDeg(0);
			sender.setVelocityPerSec(0);
			sender.setAzimuthDeg(0);
			sender.setFuturePositionCount(0);
		}
		else if(const_latitude_deg_ == 0.0 && const_longitude_deg_ == 0.0)
		{
			sender.setLatitudeDeg(gnss_direction_.fix_latitude_rad * 180.0 / M_PI);
			sender.setLongitudeDeg(gnss_direction_.fix_longitude_rad * 180.0 / M_PI);
			sender.setVelocityPerSec(vel);
			//double azi = (gnss_direction_.azimuth_rad + M_PI) * 180.0 / M_PI;
			double azi;
			if(gnss_direction_.azimuth_rad > 0) azi = gnss_direction_.azimuth_rad * 180.0 / M_PI;
			else azi = (2*M_PI + gnss_direction_.azimuth_rad) * 180.0 / M_PI;
			if(azi < AZIMITH_MIN) azi = AZIMITH_MIN;
			if(azi > AZIMITH_MAX) azi = AZIMITH_MAX;
			sender.setAzimuthDeg(azi);
			sender.setFuturePositionCount(0);
		}
		else
		{
			sender.setLatitudeDeg(const_latitude_deg_);
			sender.setLongitudeDeg(const_longitude_deg_);
			sender.setVelocityPerSec(vel);
			sender.setAzimuthDeg(0);
			sender.setFuturePositionCount(0);
		}
		

		cryptography_aes::AES aes128(aes128_key_, aes128_constract_vec_);
		std::vector<uint8_t> enc_data;
		if(aes128.encode(sender.getRawDate(), enc_data) != cryptography_aes::AES::OK)
		{
			std::cout << "error : not encode" << std::endl;
			return;
		}

		/*boost::system::error_code error;
		socket_.send_to(boost::asio::buffer(enc_data.data(), enc_data.size()), boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(ip_), port_), 0, error);
		std::cout << "data size : " << enc_data.size() << std::endl;
		//boost::asio::write(socket_, boost::asio::buffer(enc_data), error);
		if (error != 0) {
			std::cout << "送信失敗: " << error.message() << std::endl;
			return;
		}
		else {
			std::cout << "送信成功" << std::endl;
		}*/

		if(sendto(socket_, enc_data.data(), enc_data.size(), 0, (struct sockaddr *)&addr_, sizeof(addr_)) == -1){
		//if(sendto(socket_, sender.getRawDate().data(), sender.getRawDate().size(), 0, (struct sockaddr *)&addr_, sizeof(addr_)) == -1){
			std::cout << "送信失敗: " << std::endl;
			return;
		}
		else {
			std::cout << "送信成功" << std::endl;
		}

		//sender.print();
	}

	{
		if(exchange_flag_ == false) return;

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

		std_msgs::UInt8MultiArray decode_data_pub;
		decode_data_pub.data = decode_data;
		pub_decode_data_.publish(decode_data_pub);

		if(first_write_ == false)
		{
			//writeCSV("/home/sit/decode.csv", decode_data);
			first_write_ = true;
		}

		i2v::READER reader(decode_data.data());
		light_color_publish(reader);

		/////////////////////////////////////////////////////////////
		/*if(reader.getServeceInfoSize() >= use_load_id_ && use_load_id_ != 0)
		{
			ServiceLoadSignalInfo service_info = reader.getServiceInfo(use_load_id_-1);//指定した方路ID情報を取得
			if(service_info.getCarSignalInfoSize() >= use_car_signal_id_ && use_car_signal_id_!=0)
			{
				CarSignalInfo1 car_signal_info1 = service_info.getCarSignalInfo(use_car_signal_id_-1);//指定した信号ID情報を取得
				if(car_signal_info1.getInfo2Size() > 0)
				{
					if(signal_stop_line_time_ < 0)//停止線情報までの時間がない
					{
						if(exchange_flag_ == true)
						{
							CarSignalInfo2 car_signal_info2 = car_signal_info1.getInfo2(0);
							int light_color = singleCircleColor(car_signal_info2);
							publishLightColor(light_color);
						}
					}
					else//停止線情報までの時間がある
					{
						size_t info_list_size = car_signal_info1.getOutputChangeCount();
						double time_sum = 0;
						bool red_flag = false;
						for(size_t list_cou=0; list_cou<info_list_size; list_cou++)
						{
							CarSignalInfo2 car_signal_info2 = car_signal_info1.getInfo2(list_cou);
							uint16_t time = car_signal_info2.getMinRemainingSec10m();
							time_sum += time / (double)10.0;
							if(car_signal_info2.getCircleColorDisplay() == 0x3)
							{
								red_flag = true;
								break;
							}
						}

						if(red_flag == true && time_sum < config_.red_time_sec) //赤信号情報がある
						{
							publishLightColor(TRAFFIC_LIGHT_RED);
						}
						else
						{
							publishLightColor(TRAFFIC_LIGHT_GREEN);
						}
					}
				}
			}
		}*/

		/////////////////////////////////////////////////////////////
		//eader.print();
	}

	timer_count_++;
}

void I2V_Sender::callbackDecodeData(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
	i2v::READER reader(msg->data.data());
	light_color_publish(reader);
}

void I2V_Sender::release(){if(isSockOpen()) close(socket_);}
bool I2V_Sender::isSockOpen() {return socket_ok_;}
std::string I2V_Sender::getIP() {return ip_;}
int I2V_Sender::getPort() {return port_;}

void I2V_Sender::writeCSV(const std::string path, std::vector<uint8_t> &decode_data)
{
	std::ofstream ofs(path, std::ios_base::out);
	if(ofs.is_open())
	{
		for(int i=0; i<decode_data.size(); i++)
		{
			ofs << std::hex << +decode_data[i];
			if(i != decode_data.size()-1) ofs << ",";
		}
		ofs.close();
	}
}

}