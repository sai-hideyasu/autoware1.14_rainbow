/**
 * @file i2v_tcp_sender.h
 * @brief 路車間通信アプリケーション規格での信号情報を送信するクラスのヘッダーファイル
 * @author Hideyasu Sai(Saitama Institute of Technology)
 * @date 2021/12/15
 * @details 参考サイト：https://boostjp.github.io/tips/network/tcp.html
 */

#ifndef I2V_TCP_SENDER
#define I2V_TCP_SENDER

#include <ros/ros.h>
#include "sender/sender.h"
#include "reader/reader.h"
#include <cryptography_aes/aes.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <autoware_config_msgs/ConfigI2V.h>
#include <autoware_msgs/GnssDirection.h>
#include <autoware_msgs/GnssSurfaceSpeed.h>
#include <autoware_system_msgs/Date.h>
#include <autoware_msgs/Lane.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <tf/transform_datatypes.h>

/**
 * @namespace i2v
 * @brief 路車間通信アプリケーション規格での信号情報を送信するクラスのネームスペース
*/
namespace i2v
{

/**
 * @class I2V_Sender
 * @brief 路車間通信アプリケーション規格での信号情報を送信するクラス
 * @author Hideyasu Sai(Saitama Institute of Technology)
 * @date 2021/09/06
 */
class I2V_Sender
{
public:
	static const int USE_TIME_SYSTEM = 0; //!< 時間をOSから取得する
	static const int USE_TIME_GNSS = 1; //!< 時間をGNSSから取得する
private:
	const int SEND_HZ = 10; //!< 送信周期

	uint64_t guid_; //!< 送信する車両GUID
	uint32_t signalID_;// = 0x11000001; //!< 送信する宛先信号機ID
	uint32_t infomationID_;// = 0x2001; //!< 送信する信号情報種別

	const double AZIMITH_MIN = 0;
	const double AZIMITH_MAX = 359.9875;
	const size_t FUTURE_COUNT_MAX = 50;

	const double FUTURE_OFFSET_TIME_COEF = 0.001;
	const double FUTURE_OFFSET_LATITUDE_COEF = 0.0000001;
	const double FUTURE_OFFSET_LONGITUDE_COEF = 0.0000001;
	const double FUTURE_OFFSET_VELOCITY_COEF = 0.01;
	const double FUTURE_OFFSET_AZIMUTH_COEF = 0.0125;

	ros::NodeHandle nh_; //!< ROSノードハンドル(global)
	ros::NodeHandle pnh_; //!< ROSノードハンドル(private)

	ros::Subscriber sub_config_;
	ros::Subscriber sub_gnss_direction_; //!< GNSSからの緯度経度情報をsubscribe
	ros::Subscriber sub_gnss_time_; //!< GNSSからの時間をsubscribe
	ros::Subscriber sub_gnss_surface_speed_; //!< GNSSからの速度をsubscribe
	ros::Subscriber sub_signal_id_only_; //!< 信号情報のみを送信する指令をsubscribe
	ros::Subscriber sub_exchange_flag_; //!< 信号情報の提供開始指令をsubscribe
	ros::Subscriber sub_waypoint_param_;
	ros::Subscriber sub_waypoints_;
	ros::Subscriber sub_decode_data_;//!<デコードデータを読み込む

	ros::Publisher pub_signal_stat_;
	ros::Publisher pub_signal_stat_i2v_;//!<I2V情報を載せた信号情報
	ros::Publisher pub_decode_data_;

	ros::Timer timer_; //!< 送信タイマー

	//boost::asio::io_service io_; //!< OSとの仲介をするboostオブジェクト
	//boost::asio::ip::udp::socket socket_; //!< ソケット通信を行うboostオブジェクト
	int socket_;
	struct sockaddr_in addr_;
	bool socket_ok_;

	std::string ip_; //!< 送信するIPアドレス
	int port_; //!< 送信ポート番号
	std::vector<uint8_t> aes128_key_; //!< AES128の暗号鍵
	std::vector<uint8_t> aes128_constract_vec_; //!< AES128の初期化ベクトル

	autoware_config_msgs::ConfigI2V config_;
	double velocity_ms_; //!< 現在の秒速度
	autoware_msgs::GnssDirection gnss_direction_; //!< GNSSからの緯度経度情報
	autoware_system_msgs::Date gnss_date_; //!< GNSSからの時間
	uint32_t timer_count_; //!< 送信する通番　送信する毎に１増える
	uint8_t use_load_id_;
	uint8_t use_car_signal_id_;
	uint8_t use_arrow_;
	bool signal_id_only_; //!< TRUEの場合は信号情報とデータ作成日時のみ送る
	bool read_gnss_direction_flag_; //!< GNSSからの緯度経度情報を読み込むとTRUE
	bool read_gnss_time_flag_; //!< GNSSからの時間情報を読み込むとTRUE
	bool read_velocity_flag_; //!< GNSSからの速度情報を読み込むとTRUE
	bool exchange_flag_; //!< 送信する開始・終了フラグ
	int use_time_info_; //!< 使用する時間指定フラグ  USE_TIME定数軍を参照
	double const_latitude_deg_; //!< この変数かconst_longitude_deg_が0位上の場合は緯度経度の数値はこの2変数の数値で固定となる
	double const_longitude_deg_; //!< この変数かconst_latitude_deg_が0位上の場合は緯度経度の数値はこの2変数の数値で固定となる
	double const_velocity_kh_; //!< この変数が0以上の場合は速度がこの数値で固定となる(時速)
	bool first_write_;
	int prev_light_color_;//前にpublishした信号情報
	int prev_orig_color_;//前に受信したi2vの信号情報
	uint16_t prev_change_time_;
	double signal_stop_line_time_;//停止線が存在する場所までの時間
	std::string time_string_;//現在の時間をテキストにしたもの

	std::vector<uint8_t> future_data_;

	int singleCircleColor(const CarSignalInfo2 &info);

	void light_color_publish(const i2v::READER &decode_data);

	void printInit() const;

	bool execJudg() const;

	void callbackConfig(const autoware_config_msgs::ConfigI2V::ConstPtr &msg);

	void callbackGnssDirection(const autoware_msgs::GnssDirection::ConstPtr &msg);

	void callbackGnssTime(const autoware_system_msgs::Date::ConstPtr &msg);

	void callbackGnssSurfaceSpeed(const autoware_msgs::GnssSurfaceSpeed::ConstPtr &msg);

	void publishLightColor(const int light_color, const int orig_light, uint16_t change_time, 
		const std::string receive_date, const std::string send_date);

	void callbackExchangeFlag(const std_msgs::Bool::ConstPtr &msg);

	void callbackSignalIdOnly(const std_msgs::Bool::ConstPtr &msg);

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg);

	void callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg);

	void send_handler(const boost::system::error_code& error, std::size_t len);

	void callbackTimer(const ros::TimerEvent& e);

	void writeCSV(const std::string path, std::vector<uint8_t> &decode_data);

	void callbackDecodeData(const std_msgs::UInt8MultiArray::ConstPtr &msg);
public:
	I2V_Sender(const ros::NodeHandle nh, const ros::NodeHandle pnh);

	void release();
	bool isSockOpen();
	std::string getIP();
	int getPort();
};

}
#endif