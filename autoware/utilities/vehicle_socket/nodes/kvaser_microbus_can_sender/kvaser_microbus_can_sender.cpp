#include <ros/ros.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <queue>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nmea_msgs/Sentence.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <can_msgs/Frame.h>
#include <mobileye_560_660_msgs/AftermarketLane.h>
#include <mobileye_560_660_msgs/ObstacleData.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_can_msgs/MicroBusCanSenderStatus.h>
#include <autoware_can_msgs/MicroBusCanVelocityParam.h>
#include <autoware_can_msgs/MicroBusSHHV.h>
#include <autoware_can_msgs/MicroBusSPPM.h>
#include <autoware_config_msgs/ConfigMicroBusCan.h>
#include <autoware_config_msgs/ConfigVelocitySet.h>
#include <autoware_config_msgs/ConfigLocalizerSwitch.h>
#include <autoware_config_msgs/ConfigCurrentVelocityConversion.h>
#include <autoware_system_msgs/Date.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/GnssStandardDeviation.h>
#include <autoware_msgs/DifferenceToWaypointDistance.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/LocalizerMatchStat.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/StopperDistance.h>
#include <autoware_msgs/SteerOverride.h>
#include <autoware_msgs/DriveOverride.h>
#include <autoware_msgs/GnssTimeDiffCount.h>
#include <autoware_msgs/GnssSatsList.h>
#include <autoware_msgs/ImuStatus.h>
#include <autoware_msgs/TransformMobileyeObstacle.h>
#include <autoware_msgs/CarCruiseStatus.h>
#include <tf/tf.h>
#include "kvaser_can.h"
#include <time.h>
#include <fstream>
#include <pthread.h>
#include <chrono>

std::string rostime2date(ros::Time nowtime)
{
	std::chrono::system_clock::time_point p = std::chrono::system_clock::from_time_t((time_t)nowtime.sec);
	std::time_t t = std::chrono::system_clock::to_time_t(p);
	struct tm date;
	localtime_r(&t, &date);
	std::stringstream ss;
	ss << 1900 + date.tm_year << "-" << date.tm_mon+1 << "-" << date.tm_mday << "_" << date.tm_hour << "-" << date.tm_min << "-" << std::fixed << std::setprecision(4) <<(double)date.tm_sec + nowtime.nsec * 1E-9;
	return ss.str();
}

int existFile(const char* path)
{
    //pathが存在するか
    struct stat st;
    if (stat(path, &st) == 0) {
        return 1;
    }

    // ファイルかどうか
    // return (st.st_mode & S_IFMT) == S_IFREG;
	return 0;
}

std::vector<std::string> split(const std::string &string, const char sep)
{
	std::vector<std::string> str_vec_ptr;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, sep))
		str_vec_ptr.push_back(token);

	return str_vec_ptr;
}

/*std::string timeString(double time)
{
	std::stringstream str;
	double hour = floor(time/(60*60));
	time -= hour * 60*60;
	double min = floor(time / 60);
	time -= min * 60;
	str << hour << ":" << min << ":" << time;
	return str.str();
}*/

static const int SYNC_FRAMES = 50;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::PoseStamped>
    TwistPoseSync;

class PID_params
{
private:
	double accel_e_prev_velocity_, brake_e_prev_velocity_;
	double accel_e_prev_acceleration_, brake_e_prev_acceleration_;
	double accel_e_prev_distance_, brake_e_prev_distance_;
	double steer_e_prev_distance_;

	double accel_diff_sum_velocity_, brake_diff_sum_velocity_;
	double accel_diff_sum_acceleration_, brake_diff_sum_acceleration_;
	double accel_diff_sum_distance_, brake_diff_sum_distance_;
	double steer_diff_sum_distance_;

	double stroke_prev_, stop_stroke_prev_;
	int stroke_state_mode_;
public:
	const static int STROKE_STATE_MODE_ACCEL_ = 0;
	const static int STROKE_STATE_MODE_ACCEL_RELEASE = 4;
	const static int STROKE_STATE_MODE_BRAKE_ = 1;
	const static int STROKE_STATE_MODE_STOP_ = 2;
	const static int STROKE_STATE_MODE_KEEP_ = 3;

	PID_params() {}
	int init(double acc_stroke)
	{
		accel_e_prev_velocity_ = brake_e_prev_velocity_ = accel_diff_sum_velocity_ = brake_diff_sum_velocity_ = 0;
		accel_e_prev_acceleration_ = brake_e_prev_acceleration_ = accel_diff_sum_acceleration_ = brake_diff_sum_acceleration_ = 0;
		accel_e_prev_distance_ = brake_e_prev_distance_ = accel_diff_sum_distance_ = brake_diff_sum_distance_ = 0;
		steer_e_prev_distance_ = steer_diff_sum_distance_ = 0;
		stroke_prev_ = stop_stroke_prev_ = acc_stroke;
		stroke_state_mode_ = PID_params::STROKE_STATE_MODE_ACCEL_;
	}

	void clear_diff_velocity()
	{
		accel_diff_sum_velocity_ = brake_diff_sum_velocity_ = 0;
	}

	void clear_diff_acceleration()
	{
		accel_diff_sum_acceleration_ = brake_diff_sum_acceleration_ = 0;
	}

	void clear_diff_distance()
	{
		accel_diff_sum_distance_ = brake_diff_sum_distance_ = 0;
	}

	void clear_steer_diff_distance()
	{
		steer_diff_sum_distance_ = 0;
	}

	double get_accel_e_prev_velocity() {return accel_e_prev_velocity_;}
	double get_accel_e_prev_acceleration_() {return accel_e_prev_acceleration_;}
	double get_accel_e_prev_distance() {return accel_e_prev_distance_;}
	double get_brake_e_prev_velocity() {return brake_e_prev_velocity_;}
	double get_brake_e_prev_acceleration() {return brake_e_prev_acceleration_;}
	double get_brake_e_prev_distance() {return brake_e_prev_distance_;}
	double get_steer_e_prev_distance() {return steer_e_prev_distance_;}
	double get_accel_diff_sum_velocity() {return accel_diff_sum_velocity_;}
	double get_accel_diff_sum_acceleration() {return accel_diff_sum_acceleration_;}
	double get_accel_diff_sum_distance() {return accel_diff_sum_distance_;}
	double get_brake_diff_sum_velocity() {return brake_diff_sum_velocity_;}
	double get_brake_diff_sum_acceleration() {return brake_diff_sum_acceleration_;}
	double get_brake_diff_sum_distance() {return brake_diff_sum_distance_;}
	double get_steer_diff_sum_distance() {return steer_diff_sum_distance_;}
	double get_stop_stroke_prev() {return stop_stroke_prev_;}
	double get_stroke_prev() {return stroke_prev_;}
	int get_stroke_state_mode_() {return stroke_state_mode_;}

	void set_accel_e_prev_velocity(double val) {accel_e_prev_velocity_ = val;}
	void set_accel_e_prev_acceleration(double val) {accel_e_prev_acceleration_ = val;}
	void set_accel_e_prev_distance(double val) {accel_e_prev_distance_ = val;}
	void set_brake_e_prev_velocity(double val) {brake_e_prev_velocity_ = val;}
	void set_brake_e_prev_acceleration(double val) {brake_e_prev_acceleration_ = val;}
	void set_brake_e_prev_distance(double val) {brake_e_prev_distance_ = val;}
	void set_steer_e_prev_distance(double val) {steer_e_prev_distance_ = val;}
	void plus_accel_diff_sum_velocity(double val) {accel_diff_sum_velocity_ += val;}
	void plus_accel_diff_sum_acceleration(double val) {accel_diff_sum_acceleration_ += val;}
	void plus_accel_diff_sum_distance(double val) {accel_diff_sum_distance_ += val;}
	void plus_brake_diff_sum_velocity(double val) {brake_diff_sum_velocity_ += val;}
	void plus_brake_diff_sum_acceleration(double val) {brake_diff_sum_acceleration_ += val;}
	void plus_brake_diff_sum_distance(double val) {brake_diff_sum_distance_ += val;}
	void plus_steer_diff_sum_distance(double val) {steer_diff_sum_distance_ += val;}
	void set_stop_stroke_prev(double val) {stop_stroke_prev_ = val;}
	void set_stroke_prev(double val) {stroke_prev_ = val;}
	void set_stroke_state_mode_(int val) {stroke_state_mode_ = val;}
};

enum class EControl
{
  KEEP = -1,
  STOP = 1,
  STOPLINE = 2,
  DECELERATE = 3,
  OTHERS = 4,
};

class kvaser_can_sender
{
private:
	//velcity params
	//const short VELOCITY_ZERO_VALUE_ = 132;//stopping_controlで使用

	//vehicle params
	double wheelrad_to_steering_can_value_left_ = 25009.6727514125;//liesse 20935.4958411006;//cmdのwheel指令をcanのハンドル指令に変換する係数(左回り用)
	double wheelrad_to_steering_can_value_right_ = 26765.9140133745;//liesse 20791.4464661611;//cmdのwheel指令をcanのハンドル指令に変換する係数(右回り用)

	//canの送信コード0x100のdriveモード指定
	const static unsigned char MODE_STROKE   = 0x0A;//strokeモード
	const static unsigned char MODE_VELOCITY = 0x0B;//velocityモード　現在こちらは使用していない

	//shift_param
	const static unsigned char SHIFT_P = 0;
	const static unsigned char SHIFT_R = 1;
	const static unsigned char SHIFT_N = 2;
	const static unsigned char SHIFT_D = 3;
	const static unsigned char SHIFT_4 = 4;
	const static unsigned char SHIFT_L = 5;

	//other params
	const static unsigned int SEND_DATA_SIZE = 8;//canに送る1stepのデータ量(byte)

	//速度情報の選択
	const static int USE_VELOCITY_CAN = 0;//canからの速度を使用
	const static int USE_VELOCITY_TWIST = 1;//current_velocityからの速度を使用

	//ues acceleration topic
	const static int USE_ACCELERATION_TWIST1 = 0;
	const static int USE_ACCELERATION_TWIST2 = 1;
	const static int USE_ACCELERATION_IMU = 2;

	//steer_overwirte
	const static double STEER_OVERRIDE_TH = -100000;//steerのactualを上書きする判定のしきい値
	const static double DRIVE_OVERRIDE_TH = -100000;//driveのactualを上書きする判定のしきい値

	//mpc_steer_gradually_change_distance
	const static double MPC_STEER_GRADUALLY_CHANGE_DISTANCE_INIT = 3;//(m)

	//localizer更新時間のチェック時間(s)
	const static double LOCALIZER_UPDATE_CHECK_TIME = 0.4;

	//novatelのステート情報
	const std::vector<std::string> NOVATEL_STAT_STR =
		{
			"INS_INACTIVE", "INS_ALIGNING", "INS_HIGH_VARIANCE", "INS_SOLUTION_GOOD", "NONE", "NONE", "INS_SOLUTION_FREE",
			"INS_ALIGNMENT_COMPLETE", "DETERMINING_ORIENTATION", "WAITING_INITIALPOS", "WAITING_AZIMUTH", 
			"INITIALIZING_BIASES", "MOTION_DETECT", "NONE", "WAITING_ALIGNMENTORIENTATION"
		};


	ros::Publisher pub_microbus_can_sender_status_, pub_log_write_;
	ros::Publisher pub_localizer_match_stat_, pub_stroke_routine_, pub_vehicle_status_, pub_velocity_param_, pub_tmp_;
	ros::Publisher pub_brake_i_, pub_log_write_flag_, pub_acc_;

	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_microbus_drive_mode_, sub_microbus_steer_mode_, sub_twist_cmd_;
	ros::Subscriber sub_microbus_can_100_string_, sub_microbus_can_501_, sub_microbus_can_502_, sub_microbus_can_503_;
	ros::Subscriber sub_first_lock_release_, sub_stroke_mode_, sub_velocity_mode_;
	ros::Subscriber sub_input_steer_flag_, sub_input_drive_flag_, sub_input_steer_value_, sub_input_drive_value_;
	ros::Subscriber sub_waypoint_param_, sub_waypoints_, sub_position_checker_, sub_config_microbus_can_;
	ros::Subscriber sub_shift_auto_, sub_shift_position_;
	ros::Subscriber sub_emergency_stop_;
	ros::Subscriber sub_light_high_;
	ros::Subscriber sub_blinker_right_, sub_blinker_left_, sub_blinker_stop_;
	ros::Subscriber sub_automatic_door_, sub_drive_clutch_, sub_steer_clutch_;
	ros::Subscriber sub_econtrol_, sub_stopper_distance_;
	ros::Subscriber sub_lidar_detector_objects_, sub_imu_, sub_gnss_standard_deviation_, sub_gnss_standard_deviation_sub_, sub_gnss_sats_list_, sub_gnss_imu_status_;
	ros::Subscriber sub_ndt_stat_string, sub_gnss_stat_, sub_ndt_pose_, sub_gnss_pose_, sub_ndt_stat_, sub_ndt_reliability_, sub_ekf_pose_;
	ros::Subscriber sub_difference_to_waypoint_distance_, sub_difference_to_waypoint_distance_ndt_, sub_difference_to_waypoint_distance_gnss_, sub_difference_to_waypoint_distance_ekf_;
	ros::Subscriber sub_localizer_select_num_, sub_config_localizer_switch_, sub_interface_lock_;//sub_interface_config_;
	ros::Subscriber sub_ekf_covariance_, sub_use_safety_localizer_, sub_config_current_velocity_conversion_;
	ros::Subscriber sub_cruse_velocity_, sub_temporary_fixed_velocity_;
	ros::Subscriber sub_mobileye_frame_, sub_mobileye_obstacle_data_, sub_front_mobileye_car_;
	ros::Subscriber sub_antenna_pose_, sub_antenna_pose_sub_, sub_gnss_time_, sub_log_write_, sub_cruse_error_;
	ros::Subscriber sub_log_folder_, sub_waypoints_file_name_, sub_steer_override_, sub_drive_override_, sub_way_increase_distance_;
	ros::Subscriber sub_nmea_device_status_, sub_error_check_, sub_shhv_, sub_sppm_, sub_auto_log_write_;
	ros::Subscriber sub_safety_waypoints_, sub_receiver_steer_correction_, sub_record_topic_list_;
	ros::Subscriber sub_accel_stroke_cap_mobileye_, sub_accel_stroke_cap_temporary_stopper_;
	ros::Subscriber sub_car_cruise_status_;

	message_filters::Subscriber<geometry_msgs::TwistStamped> *sub_current_velocity_;
	message_filters::Subscriber<geometry_msgs::PoseStamped> *sub_current_pose_;
	message_filters::Synchronizer<TwistPoseSync> *sync_twist_pose_;

	KVASER_CAN kc;
	bool flag_drive_mode_, flag_steer_mode_;
	bool input_drive_mode_, input_steer_mode_;
	autoware_config_msgs::ConfigMicroBusCan setting_;
	autoware_config_msgs::ConfigLocalizerSwitch config_localizer_switch_;
	//autoware_config_msgs::ConfigMicrobusInterface interface_config_;
	unsigned char drive_control_mode_;
	std::string read_can100_string_;//can100の返答値を文字列にしたもの
	autoware_can_msgs::MicroBusCan501 can_receive_501_;
	autoware_can_msgs::MicroBusCan502 can_receive_502_;
	autoware_can_msgs::MicroBusCan503 can_receive_503_;
	ros::Duration diff_time_501_, diff_time_502_, diff_time_503_;
	geometry_msgs::TwistStamped current_velocity_;
	geometry_msgs::PoseStamped current_pose_;
	sensor_msgs::Imu imu_;
	double acceleration1_twist_, acceleration2_twist_, jurk1_twist_, jurk2_twist_;
	std::vector<double> acceleration_vec1_, acceleration_vec2_;
	autoware_msgs::VehicleCmd twist_;
	short input_steer_, input_drive_;
	short pedal_;
	bool shift_auto_;
	unsigned char shift_position_, drive_clutch_, steer_clutch_, automatic_door_;
	unsigned char emergency_stop_;
	bool light_high_;
	bool blinker_right_, blinker_left_, blinker_stop_, blinker_param_sender_;
	EControl econtrol;
	autoware_msgs::StopperDistance stopper_distance_;
	autoware_msgs::WaypointParam waypoint_param_;
	PID_params pid_params;
	int use_velocity_data_, use_acceleration_data_;
	//autoware_msgs::PositionChecker position_checker_;
	bool angle_limit_over_;
	double steer_correction_;
	std::string ndt_stat_string_;
	geometry_msgs::PoseStamped ndt_pose_, gnss_pose_, ekf_pose_, antenna_pose_, antenna_pose_sub_;
	unsigned char gnss_stat_;
	autoware_msgs::DifferenceToWaypointDistance difference_toWaypoint_distance_, difference_toWaypoint_distance_ndt_, difference_toWaypoint_distance_gnss_, difference_toWaypoint_distance_ekf_;
	autoware_msgs::GnssStandardDeviation gnss_deviation_, gnss_deviation_sub_;
	autoware_msgs::NDTStat ndt_stat_;
	double ndt_reliability_;
	int localizer_select_num_;
	ros::Time automatic_door_time_;
	ros::Time blinker_right_time_, blinker_left_time_, blinker_stop_time_;
	ros::Time drive_clutch_timer_, steer_clutch_timer_;
	ros::Time can_send_time_, localizer_timer_, gnss_stat_error_time_;
	double waypoint_id_ = -1;
	double ndt_gnss_angle_;
	double accel_avoidance_distance_min_, stop_stroke_max_;
	bool in_accel_mode_, in_brake_mode_;
	std_msgs::String routine_;
	bool use_stopper_distance_, interface_lock_;
	geometry_msgs::PoseWithCovarianceStamped ekf_covariance_;
	int ndt_warning_count_;
	bool use_safety_localizer_;
	autoware_config_msgs::ConfigCurrentVelocityConversion config_current_velocity_conversion_;
	double cruse_velocity_;
	mobileye_560_660_msgs::AftermarketLane mobileye_lane_;
	mobileye_560_660_msgs::ObstacleData mobileye_obstacle_data_;
	autoware_msgs::TransformMobileyeObstacle front_mobileye_obs_;//前方車両情報
	double temporary_fixed_velocity_;
	double send_step_;
	autoware_system_msgs::Date gnss_time_;
	std::string log_folder_;
	//std::string log_path_;
	std::ofstream ofs_log_writer_;
	long long int log_write_size_;
	std::string waypoints_file_name_, logwrite_waypoints_file_name_;
	double stop_distance_over_sum_, stop_distance_over_add_;
	bool  use_slow_accel_release_;
	pthread_t thread_can_send_;//通信ボードに速度、操舵情報を送るスレッド
	bool thread_can_send_run_flag_;//thread_can_send_スレッドが実行されているか
	bool first_lock_release_flag_;//emergency_resetを実行
	double steer_override_value_;//-100000以上の場合、steerのtargetをこの値に上書きする
	double drive_override_value_;//-100000以上の場合、driveのtargetをこの値に上書きする
	double last_steer_override_value_;//steerオーバーライド終了時の上書き値
	double mpc_steer_gradually_change_distance_;//steer指令を上書き状態からmpc指令に徐々に戻す距離
	double target_steer_;//現在canに送信したsteer_targetの値
	double local_way_max_vel_mps_;//現在のlocal_waypointsでのtwist上の最大速度
	autoware_can_msgs::MicroBusSHHV shhv_;//rs232のsteer応答
	autoware_can_msgs::MicroBusSPPM sppm_;//rs232のdrive応答
	bool auto_log_write_;//log自動書き込みフラグ
	std::string fail_safe_flag_;//フェイルセーフが発生したか
	bool read_local_waypoints_;//local waypontsを読み込んだか？
	unsigned char id100_send_[SEND_DATA_SIZE];//can100の現在の送信値
	double first_waypoint_yaw_;//走行経路の最初のwaypointの向き
	double first_waypoint_roll_;//走行経路の最初のwaypointの向き
	double first_waypoint_pitch_;//走行経路の最初のwaypointの向き
	unsigned int gnss_receive_count_;//GNSS受信数
	autoware_msgs::ImuStatus gnss_imu_status_;//GNSSにに使用されているIMUのステータス情報
	int16_t receiver_steer_correction_;//受信側から送られたsteer補正値
	std::string record_topic_list_;//log記録時に取るrosbag topic一覧(スペース区切り)
	bool log_write_buttom_;//インターフェース上のボタンからのlog書き込み通知
	bool log_write_501_;//501のlogトリガーからのlog書き込み通知
	int16_t accel_stroke_cap_mobileye_;//mobileyeのノードから通知されたaccel strokeのキャップ
	int16_t accel_stroke_cap_temporary_stopper_;//temporay_stopperのノードから通知されたaccel strokeのキャップ
	autoware_msgs::CarCruiseStatus car_cruise_status_;//前方車両追従情報

	ros::Publisher pub_acceleration_main, pub_acceleration_ideal, pub_list_acceleration;

	bool use_error_check_;//エラー発生時にクラッチを切るか
	bool err_distance_;//current_poseと経路の距離、角度のエラー判定の通過フラグ
	bool err_localizer_time_;//localizerの更新が一定時間行われない場合にtrue
	bool err_localizer_stat_;//ndtやgnssのステータス判定エラーのフラグ
	bool err_gnss_dev_;//gnssの誤差でのエラー

	void callbackCarCruiseStatus(const autoware_msgs::CarCruiseStatus::ConstPtr &msg)
	{
		car_cruise_status_ = *msg;
	}

	void callbackAccelStrokeCapTemporaryStopper(const std_msgs::Int16::ConstPtr &msg)
	{
		accel_stroke_cap_temporary_stopper_ = msg->data;
	}

	void callbackAccelStrokeCapMobileye(const std_msgs::Int16::ConstPtr &msg)
	{
		accel_stroke_cap_mobileye_ = msg->data;
	}

	void callbackRecordTopicList(const std_msgs::String::ConstPtr &msg)
	{
		record_topic_list_ = msg->data;
	}

	void callbackReceiverSteerCorrection(const std_msgs::Int16::ConstPtr &msg)
	{
		receiver_steer_correction_ = msg->data;
	}

	void callbackSHHV(const autoware_can_msgs::MicroBusSHHV::ConstPtr &msg)
	{
		shhv_ = *msg;
	}

	void callbackSPPM(const autoware_can_msgs::MicroBusSPPM::ConstPtr &msg)
	{
		sppm_ = *msg;
	}

	void callbackErrorCheck(const std_msgs::Bool::ConstPtr &msg)
	{
		use_error_check_ = msg->data;
	}

	void callbacNmeaDeviceStatus(const std_msgs::String::ConstPtr &msg)
	{
		if(use_error_check_ && msg->data == "error_canReceiveData")
		{
			/*if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;*/
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Danger! nmea_device_status : error_canReceiveData" << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "nmea_device_status : error_canReceiveData";
			publishStatus(safety_error_message.str());
			fail_safe_flag_ = "NmeaDeviceStatus";
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			//can_send();
		}
	}

	void callbackCruseError(const std_msgs::String::ConstPtr &msg)
	{
		if(use_error_check_)
		{
			//if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
			//	drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << msg->data << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << msg->data;
			fail_safe_flag_ = "CruseError";
			//publishStatus(safety_error_message.str());
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			//can_send();
		}
	}

	void callbackWaypointFileName(const std_msgs::String::ConstPtr &msg)
	{
		waypoints_file_name_ = msg->data;
	}

	void callbackLogFolder(const std_msgs::String::ConstPtr &msg)
	{
		time_t nowtime = time(NULL);
		tm* date = localtime(&nowtime);
		int year = date->tm_year-100+2000;
		int mou = date->tm_mon+1;
		int day = date->tm_mday;
		std::stringstream ss;
		ss << msg->data << "/" << std::setfill('0') << std::right << std::setw(2) << year << "_" << std::setw(2)<< mou << "_" << std::setw(2) << day;
		log_folder_ = ss.str();
		//log_folder_ = msg->data;
	}

	void logfileOpenOrClose()//(const bool flag)
	{
		//if(flag == true)
		if(log_write_buttom_ == true || log_write_501_ == true)
		{
			//log記録
			if(!ofs_log_writer_.is_open())
			{
				//if(existFile(log_folder_.c_str()) == 0) mkdir(log_folder_.c_str(), 0777);
				std::vector<std::string> sv = split(waypoints_file_name_, '/');
				
				std::stringstream log_str, bag_str;
				if(sv.size() == 0)
				{
					//str << log_folder_ << "/NO_WAPOINT_FILE"  << "_" << gnss_time_.year <<  "_" << +gnss_time_.month << "_" << +gnss_time_.day << "_" << +gnss_time_.hour << "_" << +gnss_time_.min << "_" << gnss_time_.sec << ".csv";
					log_str << log_folder_ << "/NO_WAPOINT_FILE"  << "_" << rostime2date(ros::Time::now()) << ".csv";
					bag_str << log_folder_ << "/NO_WAPOINT_FILE"  << "_" << rostime2date(ros::Time::now()) << ".bag";
				}
				else
				{
					log_str << log_folder_ << "/" << sv[sv.size()-1] << "_" << rostime2date(ros::Time::now()) << ".csv";
					bag_str << log_folder_ << "/" << sv[sv.size()-1] << "_" << rostime2date(ros::Time::now()) << ".bag";
				}
				ofs_log_writer_.open(log_str.str(), std::ios::out);
				
				if(ofs_log_writer_.is_open())
				{
					std::cout << "log write start : " << log_str.str() << std::endl;
					//ofs_log_writer_ << waypoints_file_name_ << "\n";
					//log_path_ = log_str.str();

					std_msgs::Bool flag;
					flag.data = true;
					pub_log_write_flag_.publish(flag);
				}

				logwrite_waypoints_file_name_ = "";
			}
		}
		else
		{
			//log終了
			if(ofs_log_writer_.is_open())
			{
				ofs_log_writer_.close();
				std::cout << "log write stop : " << std::endl;
				//log_path_ = "";
				log_write_size_ = 0;

				std_msgs::Bool flag;
				flag.data = false;
				pub_log_write_flag_.publish(flag);
			}
			//system("/home/autoware/saiko_car_ware/src/autoware/utilities/runtime_manager/scripts/log_stop.sh");
		}
	}

	void callbackAutoLogWrite(const std_msgs::Bool::ConstPtr &msg)
	{
		//auto_log_write_ = msg->data;
	}

	void callbackLogWrite(const std_msgs::Bool::ConstPtr &msg)
	{
		log_write_buttom_ = msg->data;
		logfileOpenOrClose();
	}

	void callbackGnssTime(const autoware_system_msgs::Date::ConstPtr &msg)
	{
		gnss_time_ = *msg;
	}

	void callbackTemporaryFixedVelocity(const std_msgs::Float64::ConstPtr &msg)
	{
		temporary_fixed_velocity_ = msg->data;
	}

	void callbackSafetyWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
	{
		read_local_waypoints_ = true;
	}

	bool checkMobileyeObstacleStop(ros::Time nowtime)
	{
		/*ros::Duration t = nowtime - mobileye_obstacle_data_.header.stamp;
		ros::Duration th = ros::Duration(1);
		std::cout << "mobs : " << t << "," << th << "," << (int)mobileye_obstacle_data_.obstacle_status << std::endl;
		if(t < th)
		{
			switch(mobileye_obstacle_data_.obstacle_status)
			{
				case 0://undefind
				case 1://standing
				case 2://stoped
				case 5://parked
					std::cout << "mobs" << std::endl;
					return true;
				default:
					return false;
			}
		}
		else return false;*/
		return false;
	}

	void callbackMobileyeObstacleData(const mobileye_560_660_msgs::ObstacleData::ConstPtr &msg)
	{
		mobileye_obstacle_data_ = *msg;
	}

	void callbackFrontMobileyeCar(const autoware_msgs::TransformMobileyeObstacle::ConstPtr &msg)
	{
		front_mobileye_obs_ = *msg;
	}

	const bool getMessage_bool(const unsigned char *buf, unsigned int bit)
	{
		unsigned long long mask=1;
		mask<<=bit;
		unsigned long long *msgL=(unsigned long long)buf;
		if((*msgL & mask)) return true;
		else return false;
	}

	template<typename T>
	const T getMessage_bit(const unsigned char *buf, const unsigned int lowBit, const unsigned int highBit)
	{
		const unsigned int maxBitSize=sizeof(unsigned long long)*8;
		unsigned long long *msgL=(unsigned long long)buf;
		unsigned long long val=(*msgL)<<maxBitSize-highBit-1;
		unsigned int lowPos=lowBit+(maxBitSize-highBit-1);
		val>>=lowPos;
		return (T)val;
	}

	void callbackMobileyeCan(const can_msgs::Frame &frame)
	{
		switch(frame.id)
		{
		case 0x669:
			{
				if(frame.is_error == false && frame.dlc == 8)
				{
					const unsigned char *buf = (unsigned char*)frame.data.data();
					//Lane type
					mobileye_lane_.lane_type_left = getMessage_bit<unsigned char>(&buf[0], 4, 7);
					mobileye_lane_.lane_type_right = getMessage_bit<unsigned char>(&buf[5], 4, 7);
					//ldw_available
					mobileye_lane_.ldw_available_left = getMessage_bool(&buf[0], 2);
					mobileye_lane_.ldw_available_right = getMessage_bool(&buf[5], 2);
					//lane_confidence
					mobileye_lane_.lane_confidence_left = getMessage_bit<unsigned char>(&buf[0], 0, 1);
					mobileye_lane_.lane_confidence_right = getMessage_bit<unsigned char>(&buf[5], 0, 1);
					//distance_to lane
					int16_t distL, distR;
					unsigned char* distL_p = (unsigned char*)&distL;
					distL_p[1] = getMessage_bit<unsigned char>(&buf[2], 4, 7);
					distL_p[0] = getMessage_bit<unsigned char>(&buf[2], 0, 3) << 4;
					distL_p[0] |= getMessage_bit<unsigned char>(&buf[1], 4, 7);
					if(distL_p[1] & 0x8)//12bitのマイナスか
					{
						distL--;
						distL = ~distL;
						distL_p[1] &= 0x0F;
						distL = -distL;
					}
					mobileye_lane_.distance_to_left_lane = distL * 0.02;
					//std::cout << "distL : " << (int)distL << std::endl;
					unsigned char* distR_p = (unsigned char*)&distR;
					distR_p[1] = getMessage_bit<unsigned char>(&buf[7], 4, 7);
					distR_p[0] = getMessage_bit<unsigned char>(&buf[7], 0, 3) << 4;
					distR_p[0] |= getMessage_bit<unsigned char>(&buf[6], 4, 7);
					if(distR_p[1] & 0x8)//12bitのマイナスか
					{
						distR--;
						distR = ~distR;
						distR_p[1] &= 0x0F;
						distR = -distR;
					}
					mobileye_lane_.distance_to_right_lane = distR * 0.02;
					//std::cout << "distR : " << (int)distR << std::endl;
				}
				break;
			}
		}
	}

	void callbackCurrentVelocityConversion(const autoware_config_msgs::ConfigCurrentVelocityConversion::ConstPtr &msg)
	{
		config_current_velocity_conversion_ = *msg;
	}

	void callbackCruseVelocity(const std_msgs::Float64::ConstPtr &msg)
	{
		cruse_velocity_ = msg->data;
	}

	void callbackUseSafetyLocalizer(const std_msgs::Bool::ConstPtr &msg)
	{
		use_safety_localizer_ = msg->data;
		//steer_clutch_ = false;
		//std::cout << "use_safety : " << (int)use_safety_localizer_ << std::endl;
	}

	void callbackInterfaceLock(const std_msgs::BoolConstPtr &msg)
	{
		interface_lock_ = msg->data;
	}

	const int LOCALIZER_SELECT_NDT = 0;
	const int LOCALIZER_SELECT_GNSS = 1;
	void callbackLocalizerSelectNum(const std_msgs::Int32::ConstPtr msg)
	{
		localizer_select_num_ = msg->data;

		if(use_error_check_ && localizer_select_num_ < 0 && use_safety_localizer_ == true)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Danger! localizer change error" << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "localizer change error";
			publishStatus(safety_error_message.str());
			fail_safe_flag_ = "LocalizerSelectNum";
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			//can_send();
		}
	}

	void waypointDistanceCheck(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg, std::string pose_name)
	{
		if(use_error_check_ && (fabs(msg->baselink_distance) > setting_.check_distance_th ||//setting_.difference_to_waypoint_distance ||
			fabs(msg->baselink_angular) > setting_.check_angular_th) &&
			use_safety_localizer_ == true)
		{
			if(can_receive_502_.clutch == true)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				//flag_drive_mode_ = false;
				//flag_steer_mode_ = false;
				shift_auto_ = false;
				//system("aplay -D plughw:PCH /home/autoware/one33.wav");
				//can_send();
				//err_distance_ = true;
			}
			//else err_distance_ = false;
			std::cout << "Danger! " << pose_name << " : distance : " << msg->baselink_distance << "  angular : " << msg->baselink_angular << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << pose_name << "\ndistance," << msg->baselink_distance << "\nangular," << msg->baselink_angular;
			publishStatus(safety_error_message.str());
			err_distance_ = true;
			fail_safe_flag_ = "DistanceCheck";
		}
		else err_distance_ = false;
	}

	double way_distance_sum_ = 0;
	int way_distance_count_ = 0;
	void callbackDifferenceToWaypointDistance(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		way_distance_sum_ += msg->baselink_distance;
		way_distance_count_++;
		waypointDistanceCheck(msg, std::string("current"));
		difference_toWaypoint_distance_ = *msg;
	}

	double way_distance_sum_ndt_ = 0;
	int way_distance_count_ndt_ = 0;
	void callbackDifferenceToWaypointDistanceNdt(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		way_distance_sum_ndt_ += msg->baselink_distance;
		way_distance_count_ndt_++;
		if(config_localizer_switch_.localizer_check == 0 || config_localizer_switch_.localizer_check == 2)
			waypointDistanceCheck(msg, std::string("ndt"));
		difference_toWaypoint_distance_ndt_ = *msg;
	}

	double way_distance_sum_gnss_ = 0;
	int way_distance_count_gnss_ = 0;
	void callbackDifferenceToWaypointDistanceGnss(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		way_distance_sum_gnss_ += msg->baselink_distance;
		way_distance_count_gnss_++;
		if(config_localizer_switch_.localizer_check == 1 || config_localizer_switch_.localizer_check == 2)
			waypointDistanceCheck(msg, std::string("gnss"));
		difference_toWaypoint_distance_gnss_ = *msg;
	}

	double way_distance_sum_ekf_ = 0;
	int way_distance_count_ekf_ = 0;
	void callbackDifferenceToWaypointDistanceEkf(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		way_distance_sum_ekf_ += msg->baselink_distance;
		way_distance_count_ekf_++;
		//if(config_localizer_switch_.localizer_check == 1 || config_localizer_switch_.localizer_check == 2)
		//	waypointDistanceCheck(msg, std::string("gnss"));
		difference_toWaypoint_distance_ekf_ = *msg;
	}
	
	void NdtGnssCheck(ros::Time nowtime)
	{
		bool flag = true;
		//if(localizer_select_num_ == 0 && gnss_stat_ != 3) flag = false;
		//if(ndt_stat_string != "NDT_OK") flag = false;

		double ndtx = ndt_pose_.pose.position.x;
		double ndty = ndt_pose_.pose.position.y;
		double gnssx = gnss_pose_.pose.position.x;
		double gnssy = gnss_pose_.pose.position.y;
		double diff_x = ndtx - gnssx;
		double diff_y = ndty - gnssy;
		double distance = sqrt(diff_x * diff_x + diff_y* diff_y);

		autoware_msgs::LocalizerMatchStat lms;
		lms.header.stamp = nowtime;//ros::Time::now();

		//gnssステートのチェック
		//std::string gnss_stat_string = (gnss_stat_ == 3) ? "GNSS_OK" : "GNSS_ERROR";
		std::string gnss_stat_string;
		if(gnss_stat_error_time_ == ros::Time(0)) gnss_stat_string = "GNSS_OK";
		else
		{
			if(gnss_stat_error_time_ < nowtime) gnss_stat_string = "GNSS_OK";
			else gnss_stat_string = "GNSS_ERROR";
		}
		
		std::cout << "stat : " << ndt_stat_string_ << "," << gnss_stat_string << std::endl;

		if(config_localizer_switch_.localizer_check == 2)
		{
			if(use_error_check_ && fabs(difference_toWaypoint_distance_gnss_.baselink_distance - difference_toWaypoint_distance_ndt_.baselink_distance) > setting_.ndt_gnss_max_distance_limit &&
			   use_safety_localizer_ == true)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				//flag_drive_mode_ = false;
				//flag_steer_mode_ = false;
				shift_auto_ = false;
				std::cout << "Danger! difference_toWaypoint_distance_gnss : " << difference_toWaypoint_distance_ndt_.front_baselink_distance << "," << difference_toWaypoint_distance_gnss_.baselink_distance<< std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "not difference_toWaypoint\ndistance_gnss\n" << difference_toWaypoint_distance_ndt_.front_baselink_distance << "," << difference_toWaypoint_distance_gnss_.baselink_distance;
				publishStatus(safety_error_message.str());
				fail_safe_flag_ = "NdtGnssCheck1";
				//system("aplay -D plughw:PCH /home/autoware/one33.wav");
				//can_send();
			}
		}
		/*if(setting_.ndt_gnss_min_distance_limit <= distance)
		{
			ndt_gnss_difference_stat = true;
		}
		else if(setting_.ndt_gnss_max_distance_limit <= distance)
		{
			if( waypoint_localizer_angle_deg > 180 - setting_.ndt_gnss_angle_limit 
			   || waypoint_localizer_angle_deg < -180 +  setting_.ndt_gnss_angle_limit)
			   {
					ndt_gnss_difference_stat = true;
			   }
		}*/

		lms.localizer_stat = false;
		switch(config_localizer_switch_.localizer_check)
		{
			case 0://ndt only
			{
				if(ndt_stat_string_ == "NDT_OK")
				{
					ndt_warning_count_ = 0;
					lms.localizer_stat = true;
				}
				if(ndt_stat_string_ == "NDT_WARNING")
				{
					ndt_warning_count_++;
					if(ndt_warning_count_ < 50) lms.localizer_stat = true;
				}
				break;
			}
			case 1://gnss only
			{
				if(gnss_stat_string == "GNSS_OK") lms.localizer_stat = true;
				break;
			}
			case 2://ndt and gnss
			{
				if(ndt_stat_string_ == "NDT_OK" && gnss_stat_string == "GNSS_OK") lms.localizer_stat = true;
				break;
			}
		}

		/*if(localizer_select_num_ == 1 && ndt_stat_string_ == "NDT_OK" && gnss_stat_string == "GNSS_OK")// ndt_gnss_difference_stat)
		{
			lms.localizer_stat = true;
			//lms.localizer_distance = distance;
		}
		else if(localizer_select_num_ == 0 && ndt_stat_string_ == "NDT_OK")
		{
			lms.localizer_stat = true;
		}
		else*/

		if(use_error_check_ && lms.localizer_stat == false && use_safety_localizer_ == true)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Danger! not OK : " << ndt_stat_string_ << "," << gnss_stat_string << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "not OK : " << ndt_stat_string_ << "," << gnss_stat_string;
			publishStatus(safety_error_message.str());
			fail_safe_flag_ = "NdtGnssCheck2";
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			//can_send();
			//lms.localizer_stat = false;
			//lms.localizer_distance = distance;
			err_localizer_stat_ = true;
		}
		else err_localizer_stat_ = false;
		pub_localizer_match_stat_.publish(lms);

		/*if(flag == false)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Danger! Ndt Gnss check : " << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "Ndt Gnss error : ";
			publishStatus(safety_error_message.str());
			can_send();
		}*/

		if(mobileye_lane_.lane_type_left != mobileye_560_660_msgs::AftermarketLane::LANE_TYPE_NONE &&
           mobileye_lane_.lane_confidence_left >= 2 && use_safety_localizer_ == true && setting_.use_lane_left == true)
		{
			if(use_error_check_ && mobileye_lane_.distance_to_left_lane < setting_.lane_th_left)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				shift_auto_ = false;
				std::cout << "Danger! left lane : " << "," << mobileye_lane_.distance_to_left_lane << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "left lane : " << setting_.lane_th_left << "," << mobileye_lane_.distance_to_left_lane;
				publishStatus(safety_error_message.str());
				fail_safe_flag_ = "NdtGnssCheck3";
			}
		}

		if(mobileye_lane_.lane_type_right != mobileye_560_660_msgs::AftermarketLane::LANE_TYPE_NONE &&
           mobileye_lane_.lane_confidence_right >= 2 && use_safety_localizer_ == true && setting_.use_lane_right == true)
		{
			if(use_error_check_ && mobileye_lane_.distance_to_right_lane > setting_.lane_th_right)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				shift_auto_ = false;
				std::cout << "Danger! right lane : " << "," << mobileye_lane_.distance_to_right_lane << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "right lane : " << setting_.lane_th_right << "," << mobileye_lane_.distance_to_right_lane;
				publishStatus(safety_error_message.str());
				fail_safe_flag_ = "NdtGnssCheck4";
			}
		}
	}

	void callbackNdtPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		ndt_pose_ = *msg;
		//NdtGnssCheck();
	}

	void callbackGnssPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		gnss_pose_ = *msg;
		//NdtGnssCheck();
	}

	void callbackEkfPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		ekf_pose_ = *msg;
		//NdtGnssCheck();
	}

	void callbackAntennaPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		antenna_pose_ = *msg;
		//NdtGnssCheck();
	}

	void callbackAntennaPoseSub(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		antenna_pose_sub_ = *msg;
		//NdtGnssCheck();
	}

	void callbackEkfCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
	{
		ekf_covariance_ = *msg;
	}

	void callbackNdtStatString(const std_msgs::String::ConstPtr &msg)
	{
		ndt_stat_string_ = msg->data;
	}

	void callbackNdtStat(const autoware_msgs::NDTStat::ConstPtr &msg)
	{
		ndt_stat_ = *msg;
	}

	void callbackNdtReliability(const std_msgs::Float32::ConstPtr &msg)
	{
		ndt_reliability_ = msg->data;
	}

	void callbackGnssStat(const std_msgs::UInt8::ConstPtr &msg)
	{
		if(msg->data != 3)//SOLUTION_GOOD
		{
			if(gnss_stat_ == 3)
			{
				ros::Time nowtime = ros::Time::now();
				gnss_stat_error_time_ = ros::Time(nowtime.sec + 1, 0);
			}
		}
		else gnss_stat_error_time_ = ros::Time(0);
		gnss_stat_ = msg->data;
	}

	void callbackGnssStandardDeviation(const autoware_msgs::GnssStandardDeviation::ConstPtr &msg)
	{
		std::cout << "gnss_lat : " << msg->lat_std_dev << "," << setting_.gnss_lat_limit << std::endl;
		std::cout << "gnss_lon : " << msg->lon_std_dev << "," << setting_.gnss_lon_limit << std::endl;
		std::cout << "gnss_alt : " << msg->alt_std_dev << "," << setting_.gnss_alt_limit << std::endl;

		if((config_localizer_switch_.localizer_check == 1 || config_localizer_switch_.localizer_check == 2) && localizer_select_num_ == 1)
		{
			if(use_error_check_ && (msg->lat_std_dev > setting_.gnss_lat_limit ||
		        msg->lon_std_dev > setting_.gnss_lon_limit ||
		        msg->alt_std_dev > setting_.gnss_alt_limit && localizer_select_num_ == LOCALIZER_SELECT_GNSS) &&
				can_receive_502_.clutch == true &&
				use_safety_localizer_ == true)
			{
				//if(can_receive_502_.clutch == true)
				{
					if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
						drive_clutch_ = false;
					if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
						steer_clutch_ = false;
					//flag_drive_mode_ = false;
					//flag_steer_mode_ = false;
					shift_auto_ = false;
					fail_safe_flag_ = "GnssStandardDeviation";
					//system("aplay -D plughw:PCH /home/autoware/one33.wav");
					//can_send();
					err_gnss_dev_ = true;
				}
				//else err_gnss_dev_ = false;
				std::cout << "Danger! Gnss deviation limit over : " << msg->lat_std_dev << "," << msg->lon_std_dev << "," << msg->alt_std_dev << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "Gnss deviation error : ";// << msg->lat_std << "," << msg->lon_std << "," << msg->alt_std;
				publishStatus(safety_error_message.str());
			}
			else err_gnss_dev_ = false;
		}
		gnss_deviation_ = *msg;
	}

	void callbackGnssStandardDeviationSub(const autoware_msgs::GnssStandardDeviation::ConstPtr &msg)
	{
		gnss_deviation_ = *msg;
	}

	void callbackGnssSatsList(const autoware_msgs::GnssSatsList::ConstPtr &msg)
	{
		gnss_receive_count_ = msg->sats_list.size();
	}

	void callbackGnssImuStatus(const autoware_msgs::ImuStatus::ConstPtr &msg)
	{
		gnss_imu_status_ = *msg;
	}

	void callbackImu(const sensor_msgs::Imu::ConstPtr msg)
	{
		imu_ = *msg;
	}

	void callbackConfigLocalizerSwitch(const autoware_config_msgs::ConfigLocalizerSwitch::ConstPtr &msg)
	{
		config_localizer_switch_ = *msg;
	}

	void callbackLidarDetectorObjects (const autoware_msgs::DetectedObjectArray::ConstPtr &msg)
	{
		std::cout << "lidar detector : " << std::endl;
		std::cout << "ldo obj size : " << msg->objects.size() << std::endl;
		for(int obj_i=0; obj_i<msg->objects.size(); obj_i++)
		{
			int id = msg->objects[obj_i].id;
			geometry_msgs::Pose pose = msg->objects[obj_i].pose;
			std::cout << "ldo " << id << " pose x : " << pose.position.x << std::endl;
			std::cout << "ldo " << id << " pose y : " << pose.position.y << std::endl;
			std::cout << "ldo " << id << " pose z : " << pose.position.z << std::endl;
			std::cout << "ldo " << id << " orie x : " << pose.orientation.x << std::endl;
			std::cout << "ldo " << id << " orie y : " << pose.orientation.y << std::endl;
			std::cout << "ldo " << id << " orie z : " << pose.orientation.z << std::endl;
			std::cout << "ldo " << id << " orie w : " << pose.orientation.w << std::endl;

			geometry_msgs::Twist velocity = msg->objects[obj_i].velocity;
			std::cout << "ldo " << id << " linear x : " << velocity.linear.x << std::endl;
			std::cout << "ldo " << id << " linear y : " << velocity.linear.y << std::endl;
			std::cout << "ldo " << id << " linear z : " << velocity.linear.z << std::endl;
			std::cout << "ldo " << id << " ang x : " << velocity.angular.x << std::endl;
			std::cout << "ldo " << id << " ang y : " << velocity.angular.y << std::endl;
			std::cout << "ldo " << id << " ang z : " << velocity.angular.z << std::endl;
		}
	}

	void callbackStopperDistance(const autoware_msgs::StopperDistance::ConstPtr &msg)
	{
		stopper_distance_ = *msg;
		std::cout << "stopper distance : " << stopper_distance_.distance << "  process : " << +stopper_distance_.send_process << "velocity : "  << stopper_distance_.fixed_velocity << std::endl;
	}

	void callbackEmergencyStop(const std_msgs::UInt8::ConstPtr &msg)
	{
		emergency_stop_ = msg->data;
		std::cout << "emergency stop : " << (int)emergency_stop_ << std::endl;
	}

	void callbackEControl(const std_msgs::Int8::ConstPtr &msg)
	{
		econtrol = (EControl)msg->data;
	}

	void callbackFirstLockRelease(const std_msgs::Empty::ConstPtr &msg)
	{
		first_lock_release_flag_ = true;
	}

	void callbackDModeSend(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string flag = (msg->data) ? "on" : "off";
		std::cout << "sub DMode : " << flag << std::endl;
		flag_drive_mode_ = msg->data;
	}

	void callbackSModeSend(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string flag = (msg->data) ? "on" : "off";
		std::cout << "sub SMode : " << flag << std::endl;
		flag_steer_mode_ = msg->data;
	}

	void callbackMicrobusCan100String(const std_msgs::String::ConstPtr &msg)
	{
		read_can100_string_ = msg->data;
	}

	void callbackMicrobusCan501(const autoware_can_msgs::MicroBusCan501::ConstPtr &msg)
	{
		std::cout << "sub can_501" << std::endl;
		if(auto_log_write_ == true)
		{
			log_write_501_ = msg->auto_log;
			logfileOpenOrClose();
		}
		diff_time_501_ = msg->header.stamp - can_receive_501_.header.stamp;
		can_receive_501_ = *msg;
	}

	void callbackMicrobusCan502(const autoware_can_msgs::MicroBusCan502::ConstPtr &msg)
	{
		std::cout << "sub can_502" << std::endl;
		if(msg->clutch==true && can_receive_502_.clutch==false)
		{
			input_steer_mode_ = false; //std::cout << "aaa" << std::endl;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
			fail_safe_flag_ = "NONE";
		}
		if(msg->clutch==false && can_receive_502_.clutch==true)
		{
			input_steer_mode_ = true;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}

		diff_time_502_ = msg->header.stamp - can_receive_502_.header.stamp;
		can_receive_502_ = *msg;
	}

	void callbackMicrobusCan503(const autoware_can_msgs::MicroBusCan503::ConstPtr &msg)
	{
		std::cout << "sub can_503" << std::endl;
		if(msg->clutch==true && can_receive_503_.clutch==false)
		{
			//drive_control_mode_ = MODE_VELOCITY;
			shift_auto_ = true;
			input_drive_mode_ = false;
			pid_params.set_stroke_prev(0);//ガチャコン対策
			pid_params.set_stop_stroke_prev(0);//ガチャコン対策
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
			fail_safe_flag_ = "NONE";
		}
		if(msg->clutch==false && can_receive_503_.clutch==true)
		{
			//drive_control_mode_ = MODE_STROKE;
			shift_auto_ = false;
			input_drive_mode_ = true;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}

		diff_time_503_ = msg->header.stamp - can_receive_503_.header.stamp;
		can_receive_503_ = *msg;
	}

	/*void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		std::cout << "current velocity : " << msg->twist.linear.x << std::endl;
		ros::Duration rostime = msg->header.stamp - current_velocity_.header.stamp;
		double td = rostime.sec + rostime.nsec * 1E-9;
		acceleration_ = (msg->twist.linear.x - current_velocity_.twist.linear.x) / td;
		std::cout << "acceleration," << acceleration_ << std::endl;
		current_velocity_ = *msg;
	}*/

	std::string ID100ToString()
	{
		std::stringstream ss;
		for(size_t i=0; i<SEND_DATA_SIZE; i++)
		{
			ss << std::hex << std::setfill('0') << std::right << std::setw(2) << +id100_send_[i];
			if(i != SEND_DATA_SIZE-1) ss << "_";
		}
		return ss.str();
	}

	void writeLog(ros::Time nowtime)
	{
		std::stringstream str,name;

		str << std::setprecision(10) ;
		//if(waypoints_file_name_ != logwrite_waypoints_file_name_) str << waypoints_file_name_;
		logwrite_waypoints_file_name_ = waypoints_file_name_;
		str << waypoints_file_name_;
		name << "waypoint_file";
		str << "," << waypoint_id_;
		name << "," << "waypoint_id";
		//str << "," <<  gnss_time_.year << "/" << +gnss_time_.month << "/" << +gnss_time_.hour << "/" << +gnss_time_.min << "/" << gnss_time_.sec;
		//str << "," << time_str;//timeString(time_str);
		str << "," << rostime2date(nowtime);
		name << "," << "time";
		str << "," << +can_receive_501_.steer_auto;
		name << "," << "steer_auto";
		str << "," << +can_receive_501_.drive_auto;
		name << "," << "drive_auto";
		str << "," << ID100ToString();
		name << "," << "ID100SEND";
		str << "," << read_can100_string_;
		name << "," << "ID100READ";
		str << "," << diff_time_501_.sec + diff_time_501_.nsec * 1E-9;
		name << "," << "501_diff_time";
		str << "," << diff_time_502_.sec + diff_time_502_.nsec * 1E-9;
		name << "," << "502_diff_time";
		str << "," << diff_time_503_.sec + diff_time_503_.nsec * 1E-9;
		name << "," << "503_diff_time";
		str << "," << fail_safe_flag_;
		name << "," << "fail_safe";
		str <<  "," << twist_.ctrl_cmd.linear_velocity * 3.6;
		name <<  "," << "twist_.ctrl_cmd.linear_velocity_3.6";
		//str <<  "," << cruse_velocity_;
		//name <<  "," << "cruse_velocity";
		str <<  "," << current_velocity_.twist.linear.x * 3.6;
		name <<  "," << "current_velocity_.twist.linear.x_3.6";
		str <<  "," << current_velocity_.twist.linear.x;
		name <<  "," << "current_velocity_.twist.linear.x";
		str << "," << (int)can_receive_503_.clutch;
		name << "," << "can_receive_503_.clutch";
		str << "," << can_receive_501_.stroke_reply;
		name << "," << "can_receive_501_.stroke_cmd";
		str << "," << can_receive_503_.pedal_displacement;
		name << "," << "can_receive_503_.stroke_joy";
		str << "," << can_receive_502_.velocity_actual;
		name << "," << "can_receive_502_.velocity_actual";
		str << "," << can_receive_502_.angle_deg;
		name << "," << "can_receive_502_.angle_deg";
		str << "," << can_receive_502_.angle_deg_acc;
		name << "," << "can_receive_502_.angle_deg_acc";
		str << "," << can_receive_502_.angle_target_voltage;
		name << "," << "can_receive_502_.angle_target_voltage";
		str << "," << shhv_.mechanism_steering_main;
		name << "," << "shhv_.mechanism_steering_angle_main";
		str << "," << shhv_.mechanism_steering_sub;
		name << "," << "shhv_.mechanism_steering_angle_sub";
		str <<  "," << acceleration2_twist_ ;
		name <<  "," << "acceleration2_twist_";
		str <<  "," << jurk2_twist_ ;
		name <<  "," << "jurk2_twist_" ;
		str <<  "," << difference_toWaypoint_distance_.baselink_angular;
		name <<  "," << "difference_toWaypoint_distance_.baselink_angular";
		str <<  "," << difference_toWaypoint_distance_.baselink_distance;//9
		name <<  "," << "difference_toWaypoint_distance_.baselink_distance";
		
	
		//str << difference_toWaypoint_distance_.front_baselink_distance <<",";
		//str << _steer_pid_control(difference_toWaypoint_distance_.front_baselink_distance) ;
	
		double mps = current_velocity_.twist.linear.x;
		double estimated_stopping_distance = (0 * 0 - mps*mps)/(2.0*acceleration2_twist_);

		//std::string gnss_stat_string = (gnss_stat_ == 3) ? "GNSS_OK" : "GNSS_ERROR";
		str << "," << stopper_distance_.distance;//10
		name << "," << "stopper_distance";
		str << "," << +stopper_distance_.send_process;//10
		name << "," << "stopper_send_process";
		str << "," << stopper_distance_.fixed_velocity;//10
		name << "," << "stopper_fixed_velocity";
		str << "," << temporary_fixed_velocity_;//10
		name << "," << "temporary_fixed_velocity_";
		str << "," << estimated_stopping_distance;
		name << "," << "estimated_stopping_distance";
		str << "," << ndt_stat_.score ;
		name << "," << "ndt_stat_.score" ;
		str << "," << ndt_reliability_ ;
		name << "," << "ndt_reliability" ;
		str << "," << ndt_stat_.exe_time;
		name << "," << "ndt_stat_.exe_time";
		str << "," << ndt_stat_string_ ; //15
		name << "," << "ndt_stat_string";
		if(NOVATEL_STAT_STR.size() <= gnss_stat_) str << "," << "INDEX_OVER";
		else str << "," << NOVATEL_STAT_STR[gnss_stat_];
		name << "," << "gnss_stat";
		str << "," <<  gnss_receive_count_;
		name << "," <<  "gnss_receive_count";
		str << "," << gnss_deviation_.lat_std_dev;
		name << "," << "gnss_deviation_.lat_std_dev";
		str << "," << gnss_deviation_.lon_std_dev;
		name << "," << "gnss_deviation_.lon_std_dev";
		str << "," << gnss_deviation_.alt_std_dev;//19
		name << "," << "gnss_deviation_.alt_std_dev";
		//str << "," << gnss_deviation_sub_.lat_std_dev;
		//name << "," << "gnss_deviation_sub_.lat_std_dev";
		//str << "," << gnss_deviation_sub_.lon_std_dev;
		//name << "," << "gnss_deviation_sub_.lon_std_dev";
		//str << "," << gnss_deviation_sub_.alt_std_dev;//22
		//name << "," << "gnss_deviation_sub_.alt_std_dev";
		str << "," << antenna_pose_.pose.position.x;
		name << "," << "antenna_pose_.pose.position.x";
		//str << "," << antenna_pose_sub_.pose.position.x;
		//name << "," << "antenna_pose_sub_.pose.position.x";
		str << "," << antenna_pose_.pose.position.y;
		name << "," << "antenna_pose_.pose.position.y";
		//str << "," << antenna_pose_sub_.pose.position.y;
		//name << "," << "antenna_pose_sub_.pose.position.y";
		str << "," << antenna_pose_.pose.position.z;
		name << "," << "antenna_pose_.pose.position.z";
		//str << "," << antenna_pose_sub_.pose.position.z;//28
		//name << "," << "antenna_pose_sub_.pose.position.z";
		tf::Quaternion antenna_qua;
		tf::quaternionMsgToTF(antenna_pose_.pose.orientation, antenna_qua);
		tf::Matrix3x3 antenna_mat(antenna_qua);
		double antenna_roll, antenna_pitch, antenna_yaw;
		antenna_mat.getRPY(antenna_roll, antenna_pitch, antenna_yaw);
		//tf::Quaternion antenna_qua_sub;
		//tf::quaternionMsgToTF(antenna_pose_sub_.pose.orientation, antenna_qua_sub);
		//tf::Matrix3x3 antenna_mat_sub(antenna_qua_sub);
		//double antenna_roll_sub, antenna_pitch_sub, antenna_yaw_sub;
		//antenna_mat_sub.getRPY(antenna_roll_sub, antenna_pitch_sub, antenna_yaw_sub);
		str << "," << antenna_roll;//29
		name << "," << "antenna_roll";
		str <<  "," << antenna_pitch;
		name <<  "," << "antenna_pitch";
		str <<  "," << antenna_yaw;
		name <<  "," << "antenna_yaw";
		//str << "," << antenna_roll_sub;
		//name << "," << "antenna_roll_sub";
		//str <<  "," << antenna_pitch_sub;
		//name <<  "," << "antenna_pitch_sub";
		//str <<  "," << antenna_yaw_sub;//34
		//name <<  "," << "antenna_yaw_sub";
		tf::Quaternion gnss_qua;
		tf::quaternionMsgToTF(gnss_pose_.pose.orientation, gnss_qua);
		tf::Matrix3x3 gnss_mat(gnss_qua);
		double gnss_roll, gnss_pitch, gnss_yaw;
		gnss_mat.getRPY(gnss_roll, gnss_pitch, gnss_yaw);
		str << "," << gnss_roll ;//35
		name << "," << "gnss_roll";
		str <<  "," << gnss_pitch ;
		name <<  "," << "gnss_pitch";
		str <<  "," << gnss_yaw;//37
		name <<  "," << "gnss_yaw";
		str << "," << gnss_imu_status_.status_bit;
		name << "," << "gnss_imu_status_bit";
		str << "," << gnss_imu_status_.isa_100c_temperature;
		name << "," << "gnss_imu_temperature";
		double ndtx = ndt_pose_.pose.position.x;
		double ndty = ndt_pose_.pose.position.y;
		
		double gnssx = gnss_pose_.pose.position.x;
		double gnssy = gnss_pose_.pose.position.y;
		double gnssz = gnss_pose_.pose.position.z;

		double ekfx = ekf_pose_.pose.position.x;
		double ekfy = ekf_pose_.pose.position.y;
		double distance = sqrt((ndtx - gnssx) * (ndtx -gnssx) + (ndty - gnssy) * (ndty -gnssy));
		str <<"," <<ndtx;//38
		name <<"," <<"ndtx";
		str <<"," <<ndty;
		name <<"," <<"ndty";
		str <<"," << gnssx;
		name <<"," << "gnssx";
		str <<"," << gnssy;
		name <<"," << "gnssy";
		str <<"," << gnssz;
		name <<"," << "gnssz";
		str <<"," << ekfx;
		name <<"," << "ekfx";
		str <<"," << ekfy;
		name <<"," << "ekfy";
		str <<"," << sqrt(ekf_covariance_.pose.covariance[0]);
		name <<"," << "sqrt(ekf_covariance_.pose.covariance[0])";
		str <<"," << sqrt(ekf_covariance_.pose.covariance[6*1+1]);
		name <<"," << "sqrt(ekf_covariance_.pose.covariance[6*1+1])";
		str <<"," << distance;//46
		name <<"," << "distance";
		str <<"," << ndt_gnss_angle_ ;
		name <<"," << "ndt_gnss_angle";
		str << "," <<first_waypoint_roll_;//49
		name << "," <<"waypoint_roll";
		str << "," <<first_waypoint_pitch_;
		name << "," <<"waypoint_pitch";
		str << "," <<first_waypoint_yaw_;
		name << "," <<"waypoint_yaw";

		str << "," << difference_toWaypoint_distance_ndt_.baselink_distance;//52
		name << "," << "difference_toWaypoint_distance_ndt_.baselink_distance";
		str << "," << difference_toWaypoint_distance_gnss_.baselink_distance;
		name << "," << "difference_toWaypoint_distance_gnss_.baselink_distance";
		str << "," << difference_toWaypoint_distance_ekf_.baselink_distance;
		name << "," << "difference_toWaypoint_distance_ekf_.baselink_distance";
		str << "," << difference_toWaypoint_distance_ndt_.baselink_distance - difference_toWaypoint_distance_gnss_.baselink_distance;
		name << "," << "difference_toWaypoint_distance_ndt_.baselink_distance-difference_toWaypoint_distance_gnss_.baselink_distance";
		str << "," << difference_toWaypoint_distance_ndt_.baselink_distance - difference_toWaypoint_distance_ekf_.baselink_distance;
		name << "," << "difference_toWaypoint_distance_ndt_.baselink_distance-difference_toWaypoint_distance_ekf_.baselink_distance";
		str << "," << difference_toWaypoint_distance_ekf_.baselink_distance - difference_toWaypoint_distance_gnss_.baselink_distance;//58
		name << "," << "difference_toWaypoint_distance_ekf_.baselink_distance-difference_toWaypoint_distance_gnss_.baselink_distance";

		tf::Quaternion ndt_q = tf::createQuaternionFromYaw(difference_toWaypoint_distance_ndt_.baselink_angular);
		tf::Quaternion ekf_q = tf::createQuaternionFromYaw(difference_toWaypoint_distance_ekf_.baselink_angular);
		tf::Quaternion gnss_q = tf::createQuaternionFromYaw(difference_toWaypoint_distance_gnss_.baselink_angular);
		tf::Quaternion q_ndt_ekf = ndt_q * ekf_q.inverse();
		tf::Quaternion q_ndt_gnss = ndt_q * gnss_q.inverse();
		tf::Quaternion q_ekf_gnss = ekf_q * gnss_q.inverse();
		tf::Matrix3x3 s_ndt_ekf(q_ndt_ekf);
		tf::Matrix3x3 s_ndt_gnss(q_ndt_gnss);
		tf::Matrix3x3 s_ekf_gnss(q_ekf_gnss);
		double ndt_gnss_yaw, ndt_gnss_roll, ndt_gnss_pitch;
		s_ndt_gnss.getRPY(ndt_gnss_roll, ndt_gnss_pitch, ndt_gnss_yaw);
		str << "," << ndt_gnss_roll;//58
		name << "," << "ndt_gnss_roll";
		str << "," << ndt_gnss_pitch;
		name << "," << "ndt_gnss_pitch";
		str << "," << ndt_gnss_yaw;
		name << "," << "ndt_gnss_yaw";
		double ndt_ekf_yaw, ndt_ekf_roll, ndt_ekf_pitch;
		s_ndt_ekf.getRPY(ndt_ekf_roll, ndt_ekf_pitch, ndt_ekf_yaw);
		str << "," << ndt_ekf_roll;
		name << "," << "ndt_ekf_roll";
		str << "," << ndt_ekf_pitch;
		name << "," << "ndt_ekf_pitch";
		str << "," << ndt_ekf_yaw;
		name << "," << "ndt_ekf_yaw";
		double ekf_gnss_yaw, ekf_gnss_roll, ekf_gnss_pitch;
		s_ekf_gnss.getRPY(ekf_gnss_roll, ekf_gnss_pitch, ekf_gnss_yaw);
		str << "," << ekf_gnss_roll;
		name << "," << "ekf_gnss_roll";
		str << "," << ekf_gnss_pitch;
		name << "," << "ekf_gnss_pitch";
		str << "," << ekf_gnss_yaw;
		name << "," << "ekf_gnss_yaw";
		str << "," << (int)can_receive_502_.clutch;
		name << "," << "can_receive_502_.clutch";
		str << "," << can_receive_501_.steering_angle_reply;
		name << "," << "can_receive_501_.steering_angle_reply";
		str << "," << can_receive_502_.angle_actual;
		name << "," << "can_receive_502_.angle_actual";
		str << "," << twist_.ctrl_cmd.steering_angle*180/M_PI;
		name << "," << "twist_.ctrl_cmd.steering_angle(deg)";
		str << "," << twist_.ctrl_cmd.linear_velocity;
		name << "," << "twist_.ctrl_cmd.linear_velocity";
		str << "," << routine_.data;
		name << "," << "routine_.data";
		str << "," << pid_params.get_stroke_prev();
		name << "," << "pid_params.get_stroke_prev";
		str << "," << pid_params.get_stop_stroke_prev();
		name << "," << "pid_params.get_stop_stroke_prev";
		str << "," << send_step_;//72
		name << "," << "send_step";

		str << "," << front_mobileye_obs_.orig_data.obstacle_id;
		name << "," << "mob-id";
		str << "," << front_mobileye_obs_.orig_data.obstacle_pos_x;
		name << "," << "mob-pos_x";
		str << "," << front_mobileye_obs_.orig_data.obstacle_pos_y;
		name << "," << "mob-pos_y";
		str << "," << +front_mobileye_obs_.orig_data.blinker_info;
		name << "," << "mob.-blinker";
		str << "," << +front_mobileye_obs_.orig_data.cut_in_and_out;
		name << "," << "mob-cut_in_and_out";
		str << "," << front_mobileye_obs_.orig_data.obstacle_rel_vel_x;
		name << "," << "mob-rel_vel_x";
		str << "," << +front_mobileye_obs_.orig_data.obstacle_type;
		name << "," << "mob-obs_type";
		str << "," << +front_mobileye_obs_.orig_data.obstacle_status;
		name << "," << "mob-obs_status";
		str << "," << std::boolalpha << front_mobileye_obs_.orig_data.obstacle_brake_lights;
		name << "," << "mob-brake_light";
		str << "," << +front_mobileye_obs_.orig_data.obstacle_valid;
		name << "," << "mob-obs_valid";
		str << "," << front_mobileye_obs_.orig_data.obstacle_length;
		name << "," << "mob-obs_length";
		str << "," << front_mobileye_obs_.orig_data.obstacle_width;
		name << "," << "mob-obs_width";
		str << "," << front_mobileye_obs_.orig_data.obstacle_age;
		name << "," << "mob-obs_age";
		str << "," << +front_mobileye_obs_.orig_data.obstacle_lane;
		name << "," << "mob-obs_lane";
		str << "," << std::boolalpha << front_mobileye_obs_.orig_data.cipv_flag;
		name << "," << "mob-cipv_flag";
		str << "," << front_mobileye_obs_.orig_data.radar_pos_x;
		name << "," << "mob-radar_pos_x";
		str << "," << front_mobileye_obs_.orig_data.radar_vel_x;
		name << "," << "mob-radar_vel_x";
		str << "," << +front_mobileye_obs_.orig_data.radar_match_confidence;
		name << "," << "mob-radar_match_confidence";
		str << "," << front_mobileye_obs_.orig_data.matched_radar_id;
		name << "," << "mob-matched_radar_id";
		str << "," << front_mobileye_obs_.orig_data.obstacle_angle_rate;
		name << "," << "mob-obs_angle_rate";
		str << "," << front_mobileye_obs_.orig_data.obstacle_scale_change;
		name << "," << "mob-obs_scale_change";
		str << "," << front_mobileye_obs_.orig_data.object_accel_x;
		name << "," << "mob-object_acc_x";
		str << "," << std::boolalpha << front_mobileye_obs_.orig_data.obstacle_replaced;
		name << "," << "mob-obs_replaced";
		str << "," << front_mobileye_obs_.orig_data.obstacle_angle;
		name << "," << "mob-obs_angle";

		std_msgs::String aw_msg;
		aw_msg.data = str.str();
		pub_log_write_.publish(aw_msg);
		//if(log_path_ != "")
		if(ofs_log_writer_.is_open() == true)
		{
			std::cout << "log file write" << std::endl;
			if(log_write_size_ == 0)
			{
				ofs_log_writer_ << name.str() << "\n";
				log_write_size_ += name.str().size()+1;
			}
			ofs_log_writer_ << str.str() << "\n";
			log_write_size_ += str.str().size()+1;
		}
	}

	double past_velocity_twist = 0.0;	//!< 前回速度
	double past_sec_twist = 0.0;	//!< 前回取得時の時間
	bool update_current_velocity_; //!<current_velocityが更新されたか？
	void TwistPoseCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
	                       const geometry_msgs::PoseStampedConstPtr &pose_msg)
	{
		localizer_timer_ = twist_msg->header.stamp;

		std::cout << "current velocity : " << twist_msg->twist.linear.x << std::endl;
		std::cout << "current pose : "      << pose_msg->pose.position.x << "," << pose_msg->pose.position.y << std::endl;

		ros::Duration rostime = twist_msg->header.stamp - current_velocity_.header.stamp;
		double td = rostime.sec + rostime.nsec * 1E-9;
		std::cout << "acc_time," << td << std::endl;

		// v^2 - v0^2 = 2ax
		const double x =
		    std::hypot(pose_msg->pose.position.x - current_pose_.pose.position.x, pose_msg->pose.position.y - current_pose_.pose.position.y);
		const double v0 = current_velocity_.twist.linear.x;
		const double v = twist_msg->twist.linear.x;
		double v_sa = v * v - v0 * v0;
		double acc = 0;
		//if(x >= 0.01) acc = (v_sa) / (2 * x);
		acceleration_vec1_.insert(acceleration_vec1_.begin(), acc);
		if(acceleration_vec1_.size() > 3) acceleration_vec1_.resize(3);
		acc = 0;
		int vec_cou;
		for(vec_cou=0; vec_cou<acceleration_vec1_.size(); vec_cou++)
		{
			acc += acceleration_vec1_[vec_cou];
		}
		acc /= vec_cou;
		double jurk = (acc - acceleration1_twist_) / td;

		double acc2 = (twist_msg->twist.linear.x - current_velocity_.twist.linear.x) / td;
		acceleration_vec2_.insert(acceleration_vec2_.begin(), acc2);
		if(acceleration_vec2_.size() > 3) acceleration_vec2_.resize(3);
		acc2 = 0;
		for(vec_cou=0; vec_cou<acceleration_vec2_.size(); vec_cou++)
		{
			acc2 += acceleration_vec2_[vec_cou];
		}
		acc2 /= vec_cou;
		double jurk2 = (acc2 - acceleration2_twist_) / td;
		std::cout << "acceleration," << acc << "," << acc2 << std::endl;
		double wheel_base = 3.935;
		//double tire_angle;
		//if(twist_.ctrl_cmd.steering_angle > 0) tire_angle = twist_.ctrl_cmd.steering_angle*wheelrad_to_steering_can_value_left_;
		//else tire_angle = twist_.ctrl_cmd.steering_angle*wheelrad_to_steering_can_value_right;

		//writeLog();

		current_velocity_ = *twist_msg;
		current_pose_ = *pose_msg;
		acceleration1_twist_ = acc;
		acceleration2_twist_ = acc2;
		jurk1_twist_ = jurk;
		jurk2_twist_ = jurk2;

		autoware_can_msgs::MicroBusCanVelocityParam vparam;
		vparam.header.stamp = ros::Time::now();
		vparam.velocity = current_velocity_.twist.linear.x;
		vparam.acceleration = acc2;
		vparam.jurk = jurk2;
		pub_velocity_param_.publish(vparam);

		//加速度表示用
		double velocity_stamp = current_velocity_.header.stamp.sec + current_velocity_.header.stamp.nsec * 1E-9;
		double accel = ((current_velocity_.twist.linear.x / 3.6 - past_velocity_twist) / (velocity_stamp - past_sec_twist));
		pub_acc_.publish(accel);
		past_velocity_twist = current_velocity_.twist.linear.x / 3.6;	//!<速度保管
		past_sec_twist = velocity_stamp;	//!< 現時刻保存

		update_current_velocity_ = true;
	}

	void callbackTwistCmd(const autoware_msgs::VehicleCmd::ConstPtr &msg)
	{
		std::cout << "sub twist" << std::endl;

		ros::Duration rostime = msg->header.stamp - twist_.header.stamp;
		double time_sa = rostime.sec + rostime.nsec * 1E-9;
		double ws_ave = (wheelrad_to_steering_can_value_left_ + wheelrad_to_steering_can_value_right_) / 2.0;
		double deg = fabs(msg->ctrl_cmd.steering_angle - twist_.ctrl_cmd.steering_angle) * ws_ave * 720.0 / 15000.0;
		double zisoku = msg->ctrl_cmd.linear_velocity * 3.6;

		bool flag = false;

		//double targetAngleTimeVal = fabs(deg - front_deg_)/time_sa;
		std::cout << "time_sa," << time_sa << ",targetAngleTimeVal," << deg << std::endl;
		double deg_th;
		if(zisoku <= 10) deg_th = setting_.steer_speed_limit1;//100;
		else deg_th = setting_.steer_speed_limit2;
		if(deg > deg_th)// && strinf.mode == MODE_PROGRAM)
		{
			if(msg->ctrl_cmd.steering_angle != 0)
			{
				flag = true;
				angle_limit_over_ = true;
				std::cout << "Danger! angular velocity over : " << deg << " th : " << deg_th << std::endl;
			}
			else angle_limit_over_ = false;
		}
		else angle_limit_over_ = false;

		//クラッチが入っている(autoモードである)場合だけエラー判定を出す
		std::stringstream safety_error_message;
		if(use_error_check_ && (flag == true ) && can_receive_502_.clutch == true && use_safety_localizer_ == true)
		{
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			safety_error_message << "angular velocity over , " << deg << "th : " << deg_th;
			//std::cout << safety_error_message.str() << std::endl;
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			//can_send();
			//fail_safe_flag_ = true;
		}

		publishStatus(safety_error_message.str());
		twist_ = *msg;

		if(zisoku >= 50)
		{
			//double  cmd_ang = msg->ctrl_cmd.steering_angle / M_PI * 180;
			
			if(twist_.ctrl_cmd.steering_angle > 0.045)
				twist_.ctrl_cmd.steering_angle = 0.045;
			if(twist_.ctrl_cmd.steering_angle < -0.045)
				twist_.ctrl_cmd.steering_angle = -0.045;
			
		}
	}

	void callbackStrokeMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub StrokeMode" << std::endl;
		drive_control_mode_ = MODE_STROKE;
	}

	void callbackVelocityMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub VelocityMode" << std::endl;
		drive_control_mode_ = MODE_VELOCITY;
	}

	void callbackShiftAuto(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string str = (msg->data == true) ? "shift_auto" : "shift_manual";
		std::cout << str << std::endl;
		shift_auto_ = msg->data;
	}

	void callbackShiftPosition(const std_msgs::UInt8::ConstPtr &msg)
	{
		std::cout << "shift position : " << (int)msg->data << std::endl;
		shift_position_ = msg->data;
	}

	void automaticDoorSet(unsigned char flag)
	{
		automatic_door_ = flag;

		ros::Time nowtime = ros::Time::now();
		automatic_door_time_ = ros::Time(nowtime.sec + 5, nowtime.nsec);
	}

	void callbackAutomaticDoor(const std_msgs::UInt8::ConstPtr &msg)
	{
		std::cout << "automatic door : " << (int)msg->data << std::endl;
		automaticDoorSet(msg->data);
	}

	/*void callbackConfigInterface(const autoware_config_msgs::ConfigMicrobusInterface::ConstPtr &msg)
	{
		interface_config_ = *msg;
	}*/

	void callbackConfigMicroBusCan(const autoware_config_msgs::ConfigMicroBusCan::ConstPtr &msg)
	{
		setting_ = *msg;
		std::string safety_error_message = "";
		publishStatus(safety_error_message);
	}

	void callbackConfigLocalizer(const autoware_config_msgs::ConfigLocalizerSwitch::ConstPtr &msg)
	{
		config_localizer_switch_ = *msg;
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		pedal_ = msg->microbus_pedal;

		if(msg->id != waypoint_param_.id)
		{
			double way_distance_ave = way_distance_sum_ / way_distance_count_;
			double way_distance_ave_ndt = way_distance_sum_ndt_ / way_distance_count_ndt_;
			double way_distance_ave_gnss = way_distance_sum_gnss_ / way_distance_count_gnss_;
			double way_distance_ave_ekf = way_distance_sum_ekf_ / way_distance_count_ekf_;
			way_distance_sum_ = 0;  way_distance_count_ = 0;
			way_distance_sum_ndt_ = 0;  way_distance_count_ndt_ = 0;
			way_distance_sum_gnss_ = 0;  way_distance_count_gnss_ = 0;
			way_distance_sum_ekf_ = 0;  way_distance_count_ekf_ = 0;
		}

		if(msg->localizer_check > 0)
		{
			config_localizer_switch_.localizer_check = msg->localizer_check;
		}

		if(msg->automatic_door == 2 && msg->automatic_door != waypoint_param_.automatic_door)
		{
			std::cout << "automatic_door : " << msg->automatic_door << std::endl;
			automaticDoorSet(2);
		}
		else if(msg->automatic_door == 1 && msg->automatic_door != waypoint_param_.automatic_door)
		{
			std::cout << "automatic_door : " << msg->automatic_door << std::endl;
			automaticDoorSet(1);
		}

		if(can_receive_502_.clutch == false && can_receive_503_.clutch == false && blinker_param_sender_ == true)
		{
			blinkerStop();
			blinker_param_sender_ = false;
		}
		else if(msg->blinker == 1 && (can_receive_502_.clutch == true || can_receive_503_.clutch == true))
		{
			blinkerLeft();
			blinker_param_sender_ = true;
		}
		else if(msg->blinker == 2 && (can_receive_502_.clutch == true || can_receive_503_.clutch == true))
		{
			blinkerRight();
			blinker_param_sender_ = true;
		}
		else if(msg->blinker == 0 && (can_receive_502_.clutch == true || can_receive_503_.clutch == true))
		{
			blinkerStop();
			blinker_param_sender_ = false;
		}

		if(msg->liesse.shift >= 0)
		{
			shift_position_ = msg->liesse.shift;
		}

		/*if(msg->steer_correction > -1000)
		{
			steer_correction_ = msg->steer_correction;
			//if(steer_correction_ > 500 || steer_correction_ < -500) steer_correction_ = 0; //足し算仕様
			if(steer_correction_ < 0.3 || steer_correction_ > 3.0) steer_correction_ = 1;//掛け算仕様
		}*/
		if(msg->steer_correction > 0)
		{
			steer_correction_ = msg->steer_correction;
		}

		if(msg->accel_stroke_offset >= 0 && msg->accel_stroke_offset <= 300)
		{ 
			setting_.accel_stroke_offset = msg->accel_stroke_offset;
		}

		if(msg->accel_avoidance_distance_min >= 0 && msg->accel_avoidance_distance_min <= 100)
		{
			accel_avoidance_distance_min_ = msg->accel_avoidance_distance_min;
			//std::cout << "kkk accel_avoidance_distance_min:  " << accel_avoidance_distance_min_ << std::endl;
		}
		if(msg->stop_stroke_max >= 300 && msg->stop_stroke_max <=500)
		{
			stop_stroke_max_ = msg->stop_stroke_max;
			//std::cout << "kkk stop_stroke_min : " << stop_stroke_max_ << std::endl;
		}
		if(msg->accel_stroke_max >= 300 && msg->accel_stroke_max <= 850)
		{
			setting_.pedal_stroke_max = msg->accel_stroke_max;
		}
		if(msg->k_accel_p_velocity >= 0 && msg->k_accel_p_velocity <= 2)
			setting_.k_accel_p_velocity = msg->k_accel_p_velocity;
		if(msg->k_accel_i_velocity >= 0 && msg->k_accel_i_velocity <= 2)
			setting_.k_accel_i_velocity = msg->k_accel_i_velocity;
		if(msg->k_accel_d_velocity >= 0 && msg->k_accel_d_velocity <= 2)
			setting_.k_accel_d_velocity = msg->k_accel_d_velocity;
		if(msg->k_brake_p_velocity >= 0 && msg->k_brake_p_velocity <= 2)
			setting_.k_brake_p_velocity = msg->k_brake_p_velocity;
		if(msg->k_brake_i_velocity >= 0 && msg->k_brake_i_velocity <= 2)
			setting_.k_brake_i_velocity = msg->k_brake_i_velocity;
		if(msg->k_brake_d_velocity >= 0 && msg->k_brake_d_velocity <= 2)
			setting_.k_brake_d_velocity = msg->k_brake_d_velocity;

		if(msg->in_accel_mode == 1) in_accel_mode_ = true;
		else in_accel_mode_ = false;
		if(msg->in_brake_mode == 1) in_brake_mode_ = true;
		else in_brake_mode_ = false;

		if(msg->use_stopper_distance == 0) use_stopper_distance_ = false;
		else use_stopper_distance_ = true;
		if(msg->stopper_distance1 > 0) setting_.stopper_distance1 = msg->stopper_distance1;
		if(msg->stopper_distance2 > 0) setting_.stopper_distance2 = msg->stopper_distance2;
		if(msg->stopper_distance3 > 0) setting_.stopper_distance3 = msg->stopper_distance3;

		if(waypoint_param_.steer_override > STEER_OVERRIDE_TH && msg->steer_override <= STEER_OVERRIDE_TH)
		{
			mpc_steer_gradually_change_distance_ = MPC_STEER_GRADUALLY_CHANGE_DISTANCE_INIT;
			last_steer_override_value_ = steer_override_value_;
		}
		steer_override_value_ = msg->steer_override;

		//if(msg->use_slow_accel_release == 1) use_slow_accel_release_ = true;
		//else use_slow_accel_release_ = false;
		waypoint_param_ = *msg;
	}

	void callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
	{
		waypoint_id_ = -1;
		first_waypoint_yaw_ = first_waypoint_roll_ = first_waypoint_pitch_ = 0;
		if(msg->waypoints.size() >= 2)
		{
			//経路の添字が０ではなく１なのは、/final_waypointsの最初のwaypointはbaselinkの位置であるため飛ばしているから
			waypoint_id_ = msg->waypoints[1].waypoint_param.id;
			tf::Quaternion qua;
			tf::quaternionMsgToTF(msg->waypoints[1].pose.pose.orientation, qua);
			tf::Matrix3x3(qua).getRPY(first_waypoint_roll_, first_waypoint_pitch_, first_waypoint_yaw_);
		}

		//PID 調整用  waypointsの最大速度を取得
		if(stopper_distance_.distance < 0)
		{
			local_way_max_vel_mps_ = 0;
			for(int wc=0; wc<msg->waypoints.size(); wc++)
			{
				double vel = msg->waypoints[wc].twist.twist.linear.x;
				if(vel > local_way_max_vel_mps_) local_way_max_vel_mps_ = vel;
			}
		}
		else local_way_max_vel_mps_ = 0;
	}

	/*void callbackPositionChecker(const autoware_msgs::PositionChecker::ConstPtr &msg)
	{
		position_checker_ = *msg;
		if(setting_.use_position_checker == true && use_safety_localizer_ == true)
		{
			if(msg->stop_flag != 0)// && can_receive_502_.clutch == true)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				//flag_drive_mode_ = false;
				//flag_steer_mode_ = false;
				shift_auto_ = false;
				std::cout << "Danger! Autoware stop flag : " << msg->stop_flag << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "positon error : " << msg->stop_flag;
				publishStatus(safety_error_message.str());
				//system("aplay -D plughw:PCH /home/autoware/one33.wav");
				can_send();
			}
			else
			{
				std::stringstream safety_error_message;
				safety_error_message << "";
				publishStatus(safety_error_message.str());
				//can_send();
			}
		}
		else
		{
			std::stringstream safety_error_message;
			safety_error_message << "";
			publishStatus(safety_error_message.str());
			//can_send();
		}
	}*/

	void publishStatus(std::string safety_error_message)
	{
		autoware_can_msgs::MicroBusCanSenderStatus msg;
		msg.header.stamp = ros::Time::now();
		msg.use_position_checker = setting_.use_position_checker;
		msg.use_input_steer = input_steer_mode_;
		msg.use_input_drive = input_drive_mode_;
		msg.use_velocity_topic = use_velocity_data_;
		//msg.position_check_stop = position_checker_.stop_flag;
		msg.angle_limit_over = angle_limit_over_;
		if(safety_error_message != "") msg.safety_error_message = safety_error_message;
		else msg.safety_error_message = "";
		std::cout << msg.safety_error_message << std::endl;
		pub_microbus_can_sender_status_.publish(msg);
	}

	void callbackInputSteerFlag(const std_msgs::Bool::ConstPtr &msg)
	{
		input_steer_mode_ = msg->data;
		std::cout << "aaa" << std::endl;
		std::string safety_error_message = "";
		publishStatus(safety_error_message);
	}

	void callbackInputSteerValue(const std_msgs::Int16::ConstPtr &msg)
	{
		input_steer_ = msg->data;
		std::cout << input_steer_ << std::endl;
	}

	void callbackInputDriveFlag(const std_msgs::Bool::ConstPtr &msg)
	{
		input_drive_mode_ = msg->data;
		std::cout << "ccc" << std::endl;
		std::string safety_error_message = "";
		publishStatus(safety_error_message);
	}

	void callbackInputDriveValue(const std_msgs::Int16::ConstPtr &msg)
	{
		input_drive_ = msg->data;
		std::cout << input_drive_ << std::endl;
	}

	void callbackLightHigh(const std_msgs::Bool::ConstPtr &msg)
	{
		light_high_ = msg->data;
		std::string str = (light_high_) ? "true" : "false";
		std::cout << "light_high : " << str << std::endl;
	}

	void callbackDriveClutch(const std_msgs::Bool::ConstPtr &msg)
	{
		drive_clutch_ = msg->data;
		ros::Time nowtime = ros::Time::now();
		drive_clutch_timer_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackSteerClutch(const std_msgs::Bool::ConstPtr &msg)
	{
		steer_clutch_ = msg->data;
		ros::Time nowtime = ros::Time::now();
		steer_clutch_timer_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void blinkerRight()
	{
		blinker_right_ = true;
		blinker_left_ = blinker_stop_ = false;
		std::cout << "blinker right" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_right_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerRight(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerRight();
		blinker_param_sender_ = false;
	}

	void blinkerLeft()
	{
		blinker_left_ = true;
		blinker_right_ = blinker_stop_ = false;
		std::cout << "blinker left" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_left_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerLeft(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerLeft();
		blinker_param_sender_ = false;
	}

	void blinkerStop()
	{
		blinker_stop_ = true;
		blinker_right_ = blinker_left_ = false;
		std::cout << "blinker stop" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_stop_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerStop(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerStop();
		blinker_param_sender_ = false;
	}

	void callbackSteerOverride(const autoware_msgs::SteerOverride::ConstPtr &msg)
	{
		steer_override_value_ = msg->steer_value;
	}

	void callbackDriveOverride(const autoware_msgs::DriveOverride::ConstPtr &msg)
	{
		drive_override_value_ = msg->drive_value;
	}

	void callbackWayIncreaseDistance(const std_msgs::Float64::ConstPtr &msg)
	{
		mpc_steer_gradually_change_distance_ -= msg->data;
		if(mpc_steer_gradually_change_distance_ < 0) mpc_steer_gradually_change_distance_ = 0;
	}

	void bufset_mode(unsigned char *buf)
	{
		unsigned char mode = 0;
		if(flag_drive_mode_ == true) mode |= drive_control_mode_;
		if(flag_steer_mode_ == true) mode |= 0xA0;
		buf[0] = mode;  buf[1] = 0;
	}

	void bufset_steer(unsigned char *buf, ros::Time nowtime)
	{
		short steer_val;
		if(input_steer_mode_ == false)
		{
			if(steer_override_value_ > STEER_OVERRIDE_TH)//steerのオーバーライドがON
			{
				steer_val = steer_override_value_;
				std::cout << "steer_override" << std::endl;
			}
			else
			{
				double wheel_ang = twist_.ctrl_cmd.steering_angle;
				double zisoku = twist_.ctrl_cmd.linear_velocity * 3.6;
				if(wheel_ang > 0)
				{
					steer_val = wheel_ang * wheelrad_to_steering_can_value_left_ * steer_correction_;
				}
				else
				{
					steer_val = wheel_ang * wheelrad_to_steering_can_value_right_ * steer_correction_;
				}
			}
			std::cout << "steer_correction : " << steer_correction_ << std::endl;
		}
		else steer_val = input_steer_;
		//steer_val -= 600;
		//PID
		double wheel_base = 3.935;
//		steer_val += _steer_pid_control(difference_toWaypoint_distance_.base_linkdistance);
//		steer_val += _steer_pid_control(wheel_base * tan(difference_toWaypoint_distance_.baselink_angular) + difference_toWaypoint_distance_.baselink_distance);
		//if(waypoint_param_.steer_pid_on > 0)
		//steer_val += _steer_pid_control(difference_toWaypoint_distance_.front_baselink_distance);
		if(can_receive_501_.steer_auto != autoware_can_msgs::MicroBusCan501::STEER_AUTO) steer_val = 0;

		unsigned char *steer_pointer = (unsigned char*)&steer_val;
		buf[2] = steer_pointer[1];  buf[3] = steer_pointer[0];
		target_steer_ = steer_val;
	}


	double _steer_pid_control(double distance)
	{
//P
		double e = distance;
		//I
		double e_i;
		pid_params.plus_steer_diff_sum_distance(e);

		if(current_velocity_.twist.linear.x < 1.0)
		{
			pid_params.clear_steer_diff_distance();
		}

		double sum_diff = pid_params.get_steer_diff_sum_distance();
		if (sum_diff > setting_.steer_max_i && sum_diff >0)
			e_i = setting_.steer_max_i;
		else if(sum_diff < setting_.steer_max_i && sum_diff <0)
			e_i = -setting_.steer_max_i;
		else
			e_i = sum_diff;

		//D
		double e_d = e - pid_params.get_steer_e_prev_distance();

		double target_steer = setting_.k_steer_p_distance* e +
		       setting_.k_steer_i_distance * e_i +
		       setting_.k_steer_d_distance * e_d;
		if(target_steer > 500) target_steer  = 500;
		if(target_steer < -500) target_steer =-500;

		pid_params.set_steer_e_prev_distance(e);
		return target_steer;
	}

	double math_stroke_kagen_accle(double current_velocity)
	{
		const double minvel = 10;
		const double maxvel = 20;
		const double minsrk = 100;
		double maxsrk = 150;
		double stroke_kagen;

		//if(current_velocity > 5)
		{
			/*if(current_velocity < minvel) stroke_kagen = 200;
			else if(current_velocity < maxvel) stroke_kagen = 175;
			else stroke_kagen = 150;*/
			stroke_kagen = 200;
		}
		//else stroke_kagen = 0;
		/*if(current_velocity > maxvel) stroke_kagen = maxsrk;
		else if(current_velocity < minvel) stroke_kagen = minsrk;
		else
		{
			double maxv = maxvel - minvel;
			double maxs = maxsrk - minsrk;
			double vel = current_velocity - minvel;
			stroke_kagen = vel*maxs/maxv + minsrk;
		}*/
		return stroke_kagen;
	}

	double math_stroke_kagen_brake(double current_velocity)
	{
		const double minvel = 10;
		const double maxvel = 20;
		const double minsrk = 0;
		double maxsrk = (stopper_distance_.distance != -1 && stopper_distance_.fixed_velocity <= 10) ? 1 : 150;
		double stroke_kagen;
		if(current_velocity > maxvel) stroke_kagen = maxsrk;
		else if(current_velocity < minvel) stroke_kagen = minsrk;
		else
		{
			double maxv = maxvel - minvel;
			double maxs = maxsrk - minsrk;
			double vel = current_velocity - minvel;
			stroke_kagen = vel*maxs/maxv + minsrk;
		}
		return stroke_kagen;
	}

	double e_d_;//PIDのD　current_velocityの更新タイミングに合わせるためにメンバ変数にしてある
	double _accel_stroke_pid_control(double current_velocity_kmh, double cmd_velocity_kmh, std::string &routine_str)
	{
		routine_str = "acc";
		stop_distance_over_sum_ = 0;
		//cmd_velocityとcurrent_velocityの差が小さい場合、stepを小さくする
		double accle_stroke_step = setting_.accel_stroke_step_max;//3;
		double vel_sa = cmd_velocity_kmh - current_velocity_kmh;
		double accel_stroke_adjust_th = (cmd_velocity_kmh + current_velocity_kmh)/2.0 * (setting_.accel_stroke_adjust_th/ 100.0);
		if(vel_sa < accel_stroke_adjust_th)
		//if(vel_sa < setting_.accel_stroke_adjust_th)
		{
			//accle_stroke_step -= (setting_.accel_stroke_adjust_th-vel_sa)*setting_.accel_stroke_step_max/setting_.accel_stroke_adjust_th;
			accle_stroke_step -= (setting_.accel_stroke_adjust_th-vel_sa)*setting_.accel_stroke_step_max/accel_stroke_adjust_th;
			if(accle_stroke_step < 0.5) accle_stroke_step = setting_.accel_stroke_step_min;
		}

		//ブレーキからアクセルに変わった場合、Iの積算値をリセット
		double stroke = setting_.pedal_center_voltage - can_receive_503_.pedal_voltage;
		std::cout << "voltage stroke : " << stroke << std::endl;
		std::cout << "voltage        : " << can_receive_503_.pedal_voltage << std::endl;
		std::cout << "brake offset   : " << setting_.brake_stroke_offset << std::endl;

		//std::cout << "if accel : " << stroke << " < " << setting_.brake_stroke_offset << std::endl;
		/*if(stroke < setting_.brake_stroke_offset)
		{
			pid_params.set_accel_e_prev_velocity(0);
			pid_params.set_accel_e_prev_acceleration(0);
			pid_params.set_stop_stroke_prev(0);
			return 0;
		}*/
		if(pid_params.get_stop_stroke_prev() > 0)
		{
			routine_str = "acc1";
			double ret = pid_params.get_stop_stroke_prev();
			ret -= 4.0;//accle_stroke_step;
			if(ret < 0) ret = 0;
			pid_params.set_stop_stroke_prev(ret);
			return -ret;
		}
		//pid_params.set_accel_e_prev_velocity(0);
		//pid_params.set_accel_e_prev_acceleration(0);
		pid_params.set_brake_e_prev_velocity(0);
		pid_params.set_brake_e_prev_acceleration(0);
		pid_params.set_stroke_prev(0);

		//P
		double e = cmd_velocity_kmh - current_velocity_kmh;
		//std::cout << "if accel : " << current_velocity << "," << cmd_velocity << "," << e << std::endl;
		//std::cout << "accel e : " << e << std::endl;
		//std::cout << "cmd vel : " << cmd_velocity << std::endl;
		//std::cout << "cur vel : " << current_velocity << std::endl;;

		//I
		double e_i;
		pid_params.plus_accel_diff_sum_velocity(e);
		e_i = std::min(pid_params.get_accel_diff_sum_velocity(), setting_.accel_max_i);
		
		if (pid_params.get_accel_diff_sum_velocity() > setting_.accel_max_i)//この判定はまだ考慮の余地あり
		{
			//routine_str = "acc2";
			//waypoints上の最大速度と現在の速度の差から、iの最大に補正をかける
			double max_i;
			double vel_diff = local_way_max_vel_mps_ * 3.6 - current_velocity_kmh;
			if(vel_diff < 10)
			{
				max_i = setting_.accel_max_i;
				routine_str = "acc2_1";
			}
			else if(vel_diff < 15)
			{
				max_i = setting_.accel_max_i * 1.2;
				routine_str = "acc2_2";
			}
			else
			{
				max_i = setting_.accel_max_i * 1.3;
				routine_str = "acc2_3";
			}
			e_i = max_i;
		}
		else
		{
			std::cout << "acc3step," << accle_stroke_step << std::endl;
			routine_str = "acc3";
			e_i = pid_params.get_accel_diff_sum_velocity();
		}

		//D
		if(update_current_velocity_ == true)//現在速度が更新された場合に実行
		{
			e_d_ = e - pid_params.get_accel_e_prev_velocity();
			update_current_velocity_ = false;
		}

		//前方車両の距離に対するPID_P
		const double front_car_distance_kp = 10.0;
		double front_car_distnace_stroke_plus = 0; 
		if((car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_ACCELERATION
			|| car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION1
			|| car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION2)
			&& car_cruise_status_.distance_x_m != -1)
		{
			front_car_distnace_stroke_plus = (car_cruise_status_.distance_rss_m - car_cruise_status_.distance_x_m) * front_car_distance_kp;
			front_car_distnace_stroke_plus = std::min(std::max(front_car_distnace_stroke_plus, -200.0), 200.0);
		}

		//PID計算
		double target_accel_stroke = setting_.k_accel_p_velocity * e +
		       setting_.k_accel_i_velocity * e_i +
		       setting_.k_accel_d_velocity * e_d_+
			   front_car_distnace_stroke_plus +
			   100;
		pid_params.set_accel_e_prev_velocity(e);

		double ret = target_accel_stroke;
		if(ret > setting_.pedal_stroke_max)
			ret = setting_.pedal_stroke_max;
		else if (ret < setting_.pedal_stroke_center)
			ret = setting_.pedal_stroke_center;
		if(ret < setting_.accel_stroke_offset) ret = setting_.accel_stroke_offset;

		if(pid_params.get_stroke_prev() < 0 && ret >= 0)
		{
			double tmp = pid_params.get_stroke_prev() + accle_stroke_step;
			if(tmp < ret) ret = tmp;
		}
		std::cout << "acc_ret," << target_accel_stroke << "," << e << "," << setting_.k_accel_p_velocity * e << "," << e_i << "," << setting_.k_accel_i_velocity * e_i << "," << e_d_ << "," << setting_.k_accel_d_velocity * e_d_ << "," << cmd_velocity_kmh << "," << current_velocity_kmh << std::endl;

		//ブレーキをゆっくり踏む
		/*if(pid_params.get_stroke_prev() < 0.0 && pid_params.get_stroke_prev() < ret)
		{
			double tmp = pid_params.get_stroke_prev() - accle_stroke_step;
			if(tmp > ret) ret = tmp;
			if(ret < setting_.pedal_stroke_min) ret = setting_.pedal_stroke_min;
		}*/

		double stroke_kagen = math_stroke_kagen_accle(current_velocity_kmh);
		if(ret < stroke_kagen) ret = stroke_kagen;
		if(ret > waypoint_param_.accel_stroke_cap) ret = waypoint_param_.accel_stroke_cap;
		if(ret > accel_stroke_cap_mobileye_) ret = accel_stroke_cap_mobileye_;
		if(ret > accel_stroke_cap_temporary_stopper_) ret = accel_stroke_cap_temporary_stopper_;
		/*std::stringstream str_pub;
		str_pub << cmd_velocity_kmh - current_velocity_kmh << "," << setting_.k_accel_p_velocity << "*" << e << "=" << setting_.k_accel_p_velocity*e << "," << setting_.k_accel_i_velocity << "*" << e_i << "=" << setting_.k_accel_i_velocity*e_i << "," << setting_.k_accel_d_velocity << "*" << e_d_ << "=" << setting_.k_accel_d_velocity*e_d_ << "," << target_accel_stroke << "," << ret;
		pub_tmp_.publish(str_pub.str());*/
		//if(ret < 100) ret = 100;//下駄を履かせて初速を上げる
		send_step_ = accle_stroke_step;
		pid_params.set_stroke_prev(ret);
		return ret;
	}

	bool brekeLimitJugde(double current_velocity_kmh, double cmd_velocity_kmh)
	{
		if(car_cruise_status_.distance_x_m == -1)//前方車両を追跡していない
		{
			//const double velocity_magn = 0.9;
			//double stopper_distance_th = (setting_.stopper_distance1 > cmd_velocity*velocity_magn) ? setting_.stopper_distance1 : cmd_velocity*velocity_magn;
			double velocity_magn;
			if(brake_stopD_first_velocity_ > 40) velocity_magn = 1.5;
			else if(brake_stopD_first_velocity_ > 35) velocity_magn = 1.4;
			else if(brake_stopD_first_velocity_ > 30) velocity_magn = 1.3;
			else if(brake_stopD_first_velocity_ > 25) velocity_magn = 1.25;
			else if(brake_stopD_first_velocity_ > 20) velocity_magn = 1.2;
			else if(brake_stopD_first_velocity_ > 15) velocity_magn = 1.1;
			else if(brake_stopD_first_velocity_ > 10) velocity_magn = 1.1;
			else velocity_magn = 1.1;
			double stopper_distance_th = current_velocity_kmh * velocity_magn;

			if(pid_params.get_stroke_prev() > 0
				&& (stopper_distance_.distance == -1 || stopper_distance_.distance > stopper_distance_th)
				//&& current_velocity_kmh - cmd_velocity_kmh <= 10)
				&& (current_velocity_kmh - cmd_velocity_kmh) / current_velocity_kmh <= 0.5)
			{
				return true;
			}
			else return false;
		}
		else//前方車両を追跡している
		{
			if(pid_params.get_stroke_prev() > 0 //&& stopper_distance_.distance == -1
				&& car_cruise_status_.expected_collision_time >= 10
				//&& current_velocity_kmh - cmd_velocity_kmh <= 10)
				&& (current_velocity_kmh - cmd_velocity_kmh) / current_velocity_kmh <= 0.5)
			{
				return true;
			}
			else false;
		}
	}

	double e_i_val_;
	//!< okanuma
	double brake_stopD_first_velocity_;//停止判定の初期判定時の速度
	std::vector<double> brake_list_acceleration;	//!< 加速度のリスト(section_cntの個数がサイズ)
	double brake_past_velocity = 0.0;	//!< 前回速度
	double brake_past_sec = 0.0;	//!< 前回取得時の時間
//	const double distance_margin = 1.0;	//!< stopDとの距離の乖離の余裕
	const double brake_acceleration_margin = 0.1;	//!< 理想加速度との乖離の余裕
	const double brake_i_min = 0.1;	//!< brake_iの最小値(appで変更できるようにしたほうが良い)
	const double brake_i_max = 0.45;//0.47;//0.45;	//!< brake_iの最大値(appで変更できるようにしたほうが良い)
	const int brake_section_cnt = 5;//10;	//!< 移動平均区間
	//!< ここまで	okanuma
	double _brake_stroke_pid_control(double current_velocity_kmh, double cmd_velocity_kmh, double acceleration, std::string &routine_str)
	{
		routine_str = "brake";
		if(stopper_distance_.distance == -1) brake_stopD_first_velocity_ = 0;

		e_i_val_ = 0;
		double brake_stroke_step = setting_.brake_stroke_step_max;//2;
		double vel_sa = current_velocity_kmh - cmd_velocity_kmh;
		double brake_stroke_adjust_th = (current_velocity_kmh + cmd_velocity_kmh)/2 * (setting_.brake_stroke_adjust_th / 100.0);

		if(vel_sa < brake_stroke_adjust_th)
		//if(vel_sa < setting_.brake_stroke_adjust_th)
		{
			//brake_stroke_step -= (setting_.brake_stroke_adjust_th-vel_sa)*(setting_.accel_stroke_step_max-1)/setting_.brake_stroke_adjust_th;
			brake_stroke_step -= (setting_.brake_stroke_adjust_th-vel_sa)*(setting_.accel_stroke_step_max-1)/brake_stroke_adjust_th;
			if(brake_stroke_step < 1) brake_stroke_step = setting_.brake_stroke_step_min;
		}
		bool use_step_flag = true;

		//減速しすぎを防ぐためにtarget下限を1で踏みとどまらせる処理
		if(brekeLimitJugde(current_velocity_kmh, cmd_velocity_kmh) == true)
		{
			routine_str = "Brelease";
			pid_params.clear_diff_velocity();
			pid_params.clear_diff_acceleration();
			pid_params.clear_diff_distance();
			pid_params.set_accel_e_prev_velocity(0);
			pid_params.set_accel_e_prev_acceleration(0);
			
			//if(current_velocity > cmd_velocity)
			if(use_slow_accel_release_ == true)//ゆっくりアクセルを離すか？
			{
				if(brake_stopD_first_velocity_ == 0) brake_stopD_first_velocity_ = current_velocity_kmh;
				double stroke_kagen = math_stroke_kagen_brake(current_velocity_kmh);
				pub_acceleration_ideal.publish(stroke_kagen);
				std::cout << "kagen : " << stroke_kagen << std::endl;
				double stroke = pid_params.get_stroke_prev()-brake_stroke_step;
				if(stroke < stroke_kagen) stroke = stroke_kagen;

				pid_params.set_stroke_prev(stroke);

				//前方車両の距離に対するPID_P
				const double front_car_distance_kp = 2.0;
				if(stroke != 1
					&& (car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_ACCELERATION
					|| car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION1
					|| car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION2)
					&& car_cruise_status_.distance_x_m != -1)
				{
					double val = (car_cruise_status_.distance_rss_m - car_cruise_status_.distance_x_m) * front_car_distance_kp;
					val = std::min(std::max(val, -50.0), 50.0);
					stroke += val;
				}
				/*std::stringstream str_pub;
				str_pub << stroke;
				pub_tmp_.publish(str_pub.str());*/
				return stroke;
			}
			//else return pid_params.get_stroke_prev();
		}
		
		//アクセルからブレーキに変わった場合、Iの積算値をリセット
		//setting_.pedal_center_voltageをペダル中央電圧値として、現在の電圧との差が０位上の場合はアクセル側と判断する
		double stroke = setting_.pedal_center_voltage - can_receive_503_.pedal_voltage;
		if (stroke > 0)
		{
			routine_str = "brake2";
			std::cout << "ACCEL_PEDAL_STROKE_OFFSET_" << std::endl;
			//pid_params.set_brake_e_prev_velocity(0);
			//pid_params.set_brake_e_prev_acceleration(0);
			pid_params.set_accel_e_prev_velocity(0);
			pid_params.set_accel_e_prev_acceleration(0);
			pid_params.set_stroke_prev(0);
			return 0;
		}


		//velocity PID
		//P
		double e = -1 * (cmd_velocity_kmh - current_velocity_kmh);
		std::cout << "if : " << cmd_velocity_kmh << "," << current_velocity_kmh << "," << e << std::endl;
		// since this is braking, multiply -1.
		if (e > 0 && e <= 1) { // added @ 2016/Aug/29
			e = 0;
			pid_params.clear_diff_velocity();
		}
		std::cout << "e " << e << std::endl;
		//I
		double e_i;
		pid_params.plus_brake_diff_sum_velocity(e);
		if (pid_params.get_brake_diff_sum_velocity() > setting_.brake_max_i)
			e_i = setting_.brake_max_i;
		else
			e_i = pid_params.get_brake_diff_sum_velocity();

		//D
		double e_d = e - pid_params.get_brake_e_prev_velocity();

		static double brake_i = 0.4;

		//!< ブレーキの踏み代(積分項のbrake_i)を加速度によって変更する okanuma
		double velocity_stamp = current_velocity_.header.stamp.sec + current_velocity_.header.stamp.nsec * 1E-9;
		//if(pid_params.get_stop_stroke_prev() >= 180 && stopper_distance_.distance > 0)
		if(stopper_distance_.distance > 0)
		{
			routine_str = "brake3";
			if(brake_past_sec != velocity_stamp)
			{
				routine_str = "brake4";
				if(brake_past_velocity != 0.0)
				{
					routine_str = "brake5";
					//!< 時間計算
					//if(gnss_time_.sec < past_sec) list_acceleration.push_back((current_velocity/3.6 - past_velocity) / (60 - past_sec + gnss_time_.sec));
					//else list_acceleration.push_back((current_velocity/3.6 - past_velocity) / (gnss_time_.sec - past_sec));
					brake_list_acceleration.push_back(((current_velocity_kmh/3.6 - brake_past_velocity) / (velocity_stamp - brake_past_sec)));
					//!< 移動平均リストをsection_cnt分にする
					
					while(brake_list_acceleration.size() > brake_section_cnt)
					{
						brake_list_acceleration.erase(brake_list_acceleration.begin());
					}
					//!< section_cnt個数溜まったら計算を行う
					if(brake_list_acceleration.size() == brake_section_cnt)
					{
						routine_str = "brake6";
						/*std::string acc_str;
						for(int i = 0; i < section_cnt; i++)
						{
							acc_str += std::to_string(list_acceleration[i]);
							acc_str += ",";
						}*/

						//!< 加速度平均値計算
						double acceleration_ave = 0.0;
						for(double i : brake_list_acceleration)
						{
							acceleration_ave += i;
						}
						acceleration_ave /= (double)brake_list_acceleration.size();


						double ideal_acceleration = (pow(stopper_distance_.fixed_velocity, 2)-pow((current_velocity_kmh/3.6), 2)) / (2 * stopper_distance_.distance);	//!< 理想加速度計算

						//!< 理想加速度とのズレが許容範囲かチェック(抜く方はチェックしなくても良いかも?)
	/*					
						if(ideal_acceleration > acceleration_ave + acceleration_margin || ideal_acceleration < acceleration_ave - acceleration_margin)
						{
							std::cout << "error : acceleration distance too big " << std::endl;
							std::cout << "acceleration_ave : " << acceleration_ave << std::endl;
							std::cout << "ideal_acceleration : " << ideal_acceleration << std::endl;
							brake_i = 0.4;
						}
						else
	*/					
						{
							//!< 踏みまし量決定
							/*double stop_distance_coef;
							const double dis_max = 10.0, dis_min = 0.0;
							const double coef_max = 1.0, coef_min = 0.9;
							if(stopper_distance_.distance == 0) stop_distance_coef = coef_max;
							else if(stopper_distance_.distance > dis_max) stop_distance_coef = coef_min;
							else
							{
								double a = (stopper_distance_.distance - dis_min) / (dis_max - dis_min);
								stop_distance_coef = a * (coef_max - coef_min) + coef_min;
							}*/
							
							brake_i += (acceleration_ave - ideal_acceleration) * 1.0;
							pub_list_acceleration.publish(std::to_string(brake_i));	//!< デバッグ用
							if(brake_i > brake_i_max) brake_i = brake_i_max;
							if(brake_i < brake_i_min) brake_i = brake_i_min;
						}
						brake_list_acceleration.clear();

						//pub_acceleration_main.publish(acceleration_ave);	//!< デバッグ用
						//pub_acceleration_ideal.publish(ideal_acceleration);	//!< デバッグ用
	/*					//!< 加速度から求めた停止までの距離の比較で踏みしろを決める場合
						double expected_distance = (-current_velocity/3.6) / (2*acceleration_ave);
						brake_i += (expected_distance - stopper_distance_.distance) * 10;
	*/					
	/*					
						if(stopper_distance_.distance < expected_distance - distance_margin)  brake_i += 0.2;
						else if(stopper_distance_.distance > expected_distance + distance_margin) brake_i -= 0.2;
						else brake_i += (stopper_distance_.distance - expected_distance) * 10;
	*/					
					}
				}
			}

			brake_past_velocity = current_velocity_kmh / 3.6;	//!<速度保管
			brake_past_sec = velocity_stamp;	//!< 現時刻保存
		}
		else
		{
			routine_str = "brake7";
			brake_i = 0.4;
			brake_list_acceleration.clear();
			brake_past_velocity = 0;	//!<速度保管
			brake_past_sec = 0;	//!< 現時刻保存
		}
		//!< ここまで okanuma

		std::stringstream str_pub;
		//前方車両の距離に対するPID_P
		const double front_car_distance_kp = 0.5;
		double front_car_distnace_stroke_plus = DBL_MAX; 
		if((car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_ACCELERATION
			|| car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION1
			|| car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION2)
			&& car_cruise_status_.distance_x_m != -1)
		{
			front_car_distnace_stroke_plus = -(car_cruise_status_.distance_rss_m - car_cruise_status_.distance_x_m) * front_car_distance_kp;
			str_pub << "mobileye," << front_car_distnace_stroke_plus;
		}

		double distance_stroke_plus = 0;
		if(stopper_distance_.distance < car_cruise_status_.distance_x_m && stopper_distance_.distance != -1)
		{
			str_pub.clear();
			const double stop_line_kp = 2;
			double stop_line_distance_stroke_plus = DBL_MAX;
			if(car_cruise_status_.tracking_mode == autoware_msgs::CarCruiseStatus::TRACKING_NO
				&& stopper_distance_.send_process == autoware_msgs::StopperDistance::TEMPORARY_STOPPER
				&& stopper_distance_.send_process == autoware_msgs::StopperDistance::SIGNAL)
			{
				stop_line_distance_stroke_plus = -(stopper_distance_.distance_rss - stopper_distance_.distance) * stop_line_kp;
				str_pub << "stopD,stop_line_distance_stroke_plus";
			}
			if(stop_line_distance_stroke_plus != DBL_MAX) distance_stroke_plus = stop_line_distance_stroke_plus;
		}
		else
		{
			if(front_car_distnace_stroke_plus != DBL_MAX) distance_stroke_plus = front_car_distnace_stroke_plus;
		}
		pub_tmp_.publish(str_pub.str());

		//PID計算
		double target_brake_stroke = setting_.k_brake_p_velocity * e +
		        brake_i * e_i +
		        setting_.k_brake_d_velocity * e_d +
				distance_stroke_plus;
		pid_params.set_brake_e_prev_velocity(e);
		e_i_val_ = e_i;

		/*std::stringstream str_pub;
		str_pub << cmd_velocity_kmh - current_velocity_kmh << "," << setting_.k_brake_p_velocity << "*" << e << "=" << setting_.k_brake_p_velocity*e << "," << brake_i << "*" << e_i << "=" << brake_i*e_i << "," << setting_.k_brake_d_velocity << "*" << e_d_ << "=" << setting_.k_accel_d_velocity*e_d_ << "," << target_brake_stroke;
		pub_tmp_.publish(str_pub.str());*/

		/*if(use_stopper_distance_ == true && stopper_distance_.send_process == autoware_msgs::StopperDistance::OBSTACLE &&
			car_cruise_status_.expected_collision_time < 10.0)//前方車両追跡時かつ衝突時間が１０秒以内の場合、強めに踏む
		{
			routine_str = "brakeO";
			target_brake_stroke = 250;
		}
		else*/ if(use_stopper_distance_ == true &&
			((stopper_distance_.send_process == autoware_msgs::StopperDistance::TEMPORARY_STOPPER && stopper_distance_.fixed_velocity <= 0) ||
		      stopper_distance_.send_process == autoware_msgs::StopperDistance::SIGNAL ||
			  stopper_distance_.send_process == autoware_msgs::StopperDistance::OBSTACLE))
		{
			std::cout << "stopper list : "<< setting_.stopper_distance1 << "," << setting_.stopper_distance2 << "," << setting_.stopper_distance3 << std::endl;
			std::cout << "kkk stop_stroke_max : " << stop_stroke_max_ << std::endl;
			if(stopper_distance_.distance >= setting_.stopper_distance2 && stopper_distance_.distance <= setting_.stopper_distance1)
			{
				routine_str = "brake8";
				stop_distance_over_sum_ = 0;
				/*std::cout << "tbs," << target_brake_stroke;
				double d = stop_stroke - target_brake_stroke;
				if(d < 0) d = 0;
				target_brake_stroke  += d * (1 - stopper_distance_/ 20.0 );
				std::cout << ",tbs," << target_brake_stroke << ",d," << d << ",dis," << stopper_distance_ << std::endl;*/
			}
			else if(stopper_distance_.distance >= setting_.stopper_distance3 && stopper_distance_.distance <= setting_.stopper_distance2)
			{
				routine_str = "brake9";
				stop_distance_over_sum_ = 0;
				/*if(current_velocity > 5.0)
				{
					std::cout << "tbs," << target_brake_stroke;
					double d = stop_stroke - target_brake_stroke;
					if(d < 0) d = 0;
					target_brake_stroke  += d * (1 - stopper_distance_/ 20.0 );
					std::cout << ",tbs," << target_brake_stroke << ",d," << d << ",dis," << stopper_distance_ << std::endl;
				}
				else {
					target_brake_stroke = pid_params.get_stop_stroke_prev();
				}*/
				//if(temporary_fixed_velocity_ > 0)

				//const double stop_cap = 250;
				if(stopper_distance_.fixed_velocity > 0)
				{
					target_brake_stroke = pid_params.get_stop_stroke_prev();
					if(current_velocity_kmh <= 0.5)
					{
						target_brake_stroke -= brake_stroke_step;
						if(target_brake_stroke < 0) target_brake_stroke = 0;
					}
				}
				/*else //ブレーキを踏んだ後、離す量に対するキャップ処理
				{
					if(target_brake_stroke < stop_cap && pid_params.get_stop_stroke_prev() > target_brake_stroke + 10)
						target_brake_stroke = stop_cap;
				}*/
			}
			else if(stopper_distance_.distance >= 0 && stopper_distance_.distance <= setting_.stopper_distance3)
			{
				if(current_velocity_kmh < 8.0)
				{
					//if(temporary_fixed_velocity_ == 0)
					if(stopper_distance_.fixed_velocity == 0 || stopper_distance_.send_process == autoware_msgs::StopperDistance::SIGNAL)
					{
						routine_str = "brakeA";
						//target_brake_stroke = 0.0 + 500.0 * pow((2.0-distance)/2.0,0.5);
						brake_stroke_step = 0.5;
						target_brake_stroke = pid_params.get_stop_stroke_prev() / 1.005;
						//if(stopper_distance_.distance == 0) target_brake_stroke = stop_stroke_max_;
						if(stopper_distance_.distance == 0 || current_velocity_.twist.linear.x <= 0.04)
							target_brake_stroke = stop_stroke_max_;
						/*target_brake_stroke = 0.0 + stop_stroke_max_ * (setting_.stopper_distance3 - stopper_distance_.distance)/setting_.stopper_distance3;
						if(target_brake_stroke < pid_params.get_stop_stroke_prev())
							target_brake_stroke = pid_params.get_stop_stroke_prev();*/
						
						/*if(target_brake_stroke == stop_stroke_max_ && can_receive_502_.velocity_mps != 0) 
						{
							stop_distance_over_sum_ += stop_distance_over_add_;
							brake_stroke_step = 0.1;
						}
						target_brake_stroke += stop_distance_over_sum_;*/
					}
					else
					{
						routine_str = "brakeB";
						stop_distance_over_sum_ = 0;
					}
				}
				else
				{
					routine_str = "brakeC";
					stop_distance_over_sum_ = 0;
				}
			}
			else
			{
				routine_str = "brakeD";
				stop_distance_over_sum_ = 0;
			}
		}

		double ret = target_brake_stroke;//std::cout << "ret " << setting_.k_brake_p_velocity << std::endl;
		if (-ret < setting_.pedal_stroke_min)
			ret = -setting_.pedal_stroke_min;
		else if (-ret > setting_.pedal_stroke_center)
			ret = -setting_.pedal_stroke_center;

		if(use_step_flag == true)
		{
			/*if(pid_params.get_stop_stroke_prev() < 0 && ret >= 0)
			{
				double tmp = pid_params.get_stop_stroke_prev() + brake_stroke_step;
				if(tmp < ret) ret = tmp;
			}*/
			
			if(pid_params.get_stop_stroke_prev()-ret >= 50 && pid_params.get_stop_stroke_prev() >= 0)
			{
				ret = pid_params.get_stop_stroke_prev();
				ret -= brake_stroke_step;
				if(ret < 0) ret = 0;
				//std::cout << "brake_ret 1" << std::endl;
			}
			//ブレーキをゆっくり踏む
			else if(pid_params.get_stop_stroke_prev() > 0.0 && pid_params.get_stop_stroke_prev() < ret)
			{
				double tmp = pid_params.get_stop_stroke_prev() - brake_stroke_step;
				if(tmp > ret) ret = tmp;
				if(-ret < setting_.pedal_stroke_min) ret = -setting_.pedal_stroke_min;
				//std::cout << "brake_ret 2" << std::endl;
			}
			else std::cout << "brake_ret 3" << std::endl;
		}

		//if(ret < 0) ret = 0;
		//if(ret > -setting_.pedal_stroke_min) ret = -setting_.pedal_stroke_min;
		send_step_ = brake_stroke_step;
		pid_params.set_stop_stroke_prev(ret);
		double stop_max = 320;
		if(ret > stop_max) ret = stop_max;
		return -ret;
	}

	/*short _stopping_control(double current_velocity)
	{std::cout << "stop cur : " << current_velocity << std::endl;
		if (current_velocity < (VELOCITY_ZERO_VALUE_+20)/100.0)
		{
			int gain = (int)(((double)setting_.pedal_stroke_min)*can_receive_502_.cycle_time);
			std::cout << "stop  gain : " << gain << std::endl;
			std::cout << "cycle time : " << can_receive_502_.cycle_time << std::endl;
			std::cout << "stroke min : " << setting_.pedal_stroke_min << std::endl;
			double ret = pid_params.get_stop_stroke_prev() + gain;
			if((int)ret > setting_.pedal_stroke_min) ret = setting_.pedal_stroke_min;
			if((int)ret < setting_.brake_stroke_stopping_med) ret = setting_.brake_stroke_stopping_med;
			return ret;
		}
		else
		{
			return setting_.brake_stroke_stopping_med;
		}
	}*/

	double accel_release()
	{
		routine_.data = "Arelease";
		pid_params.set_accel_e_prev_velocity(0);
		pid_params.set_accel_e_prev_acceleration(0);
		pid_params.set_brake_e_prev_velocity(0);
		pid_params.set_brake_e_prev_acceleration(0);
		pid_params.set_stroke_prev(0);
		pid_params.set_stop_stroke_prev(0);
		return 1;
	}

	double _keep_control()
	{
		if(routine_.data == "accel")
		{
			pid_params.set_stop_stroke_prev(0);
			return pid_params.get_stroke_prev();
		}
		else if(routine_.data == "brake")
		{
			pid_params.set_stroke_prev(0);
			return -pid_params.get_stop_stroke_prev();
		}
		else if(routine_.data == "keep")
		{
			if(pid_params.get_stroke_prev() > 0)
			{
				double accel = pid_params.get_stroke_prev();
				accel -= 2;
				if(accel < 0) accel = 0;
				pid_params.set_stroke_prev(accel);
				return accel;
			}
			else
			{
				double brake = pid_params.get_stop_stroke_prev();
				brake -= 2;
				if(brake < 0) brake = 0;
				pid_params.set_stop_stroke_prev(brake);
				return -brake;
			}
		}
		return 0;
	}

	void bufset_drive(unsigned char *buf, double current_velocity, double acceleration, double stroke_speed)
	{
		ros::Time nowtime = ros::Time::now();

		if(can_receive_501_.drive_mode == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY)
		{
			/*short drive_val;
			if(input_drive_mode_ == false)
			{
				std::cout <<"jjj : " << twist_.ctrl_cmd.linear_velocity;
				double linearx = twist_.ctrl_cmd.linear_velocity;
				double twist_drv = linearx *3.6 * 100;
				drive_val = twist_drv;
			}
			else drive_val = input_drive_;
			if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_val = 0;
			if(can_receive_503_.clutch == false)
			{
				drive_val = can_receive_502_.velocity_actual;
				shift_auto_ = false;
			}
			unsigned char *drive_point = (unsigned char*)&drive_val;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];*/
		}
		else
		{
			if(drive_override_value_ > DRIVE_OVERRIDE_TH)
			{
				short input_stroke = (short)drive_override_value_;
				unsigned char *drive_point = (unsigned char*)&input_stroke;
				buf[4] = drive_point[1];  buf[5] = drive_point[0];
				return;
			}

			/*double cmd_velocity;
			if(input_drive_mode_ == false)
				cmd_velocity = twist_.ctrl_cmd.linear_velocity * 3.6;
			else
				cmd_velocity = input_drive_ / 100.0;*/
			double cmd_velocity = twist_.ctrl_cmd.linear_velocity * 3.6;
			if(input_drive_mode_ == true && can_receive_501_.drive_auto)
				cmd_velocity = input_drive_ / 100.0;
			if(config_current_velocity_conversion_.enable == true && can_receive_502_.clutch == false && can_receive_503_.clutch == true)
			{
				switch(config_current_velocity_conversion_.velocity_mode)
				{
					case autoware_config_msgs::ConfigCurrentVelocityConversion::VELOCITY_MODE_CONSTANT_DIRECT:
					case autoware_config_msgs::ConfigCurrentVelocityConversion::VELOCITY_MODE_CAN_DIRECT:
						cmd_velocity = cruse_velocity_;
				}
			}

			//std::cout << "cur_cmd : " << current_velocity << "," << cmd_velocity << std::endl;
			double cv_s = current_velocity /3.6;
			if(acceleration <= 0 && stopper_distance_.distance >= 0)
				std::cout << "teisi," << - cv_s*cv_s/(2.0*acceleration) << "," << stopper_distance_.distance <<  std::endl;

/*std::stringstream str_tmp;
str_tmp << "cmd_velocity," << cmd_velocity << ",current_velocity," << current_velocity << ",setting_.acceptable_velocity_variation," << setting_.acceptable_velocity_variation
        << ",stopper_distance_.distance," << stopper_distance_.distance << ",in_brake_mode," << in_brake_mode_ << ",accel_avoidance_distance_min_," << accel_avoidance_distance_min_;
std_msgs::String str_ret;
str_ret.data = str_tmp.str();
pub_tmp_.publish(str_ret);*/
			std::cout << "auto_mode" << std::endl;
			double new_stroke = 0;
			std::cout << "cur_cmd : " << current_velocity << "," << cmd_velocity << "," << setting_.velocity_limit << std::endl;
			//std::cout << "if : " << cmd_velocity << " > " << current_velocity << std::endl;
			//std::cout << "if : " << cmd_velocity << " > " "0.0" << std::endl;
			//std::cout << "if : " <<current_velocity << " > " << setting_.velocity_limit << std::endl;
			//加速判定
			std::cout << "kkk accel_avoidance_distance_min : " << accel_avoidance_distance_min_ << std::endl;

			//停止線での加速判定 0.75は40km/hで走った場合に30m前で加速を止める
			double accel_mode_avoidance_distance = (current_velocity > accel_avoidance_distance_min_) ? current_velocity * 0.75 : accel_avoidance_distance_min_;

			/*std::stringstream str_pub;
			str_pub << "tmp," << std::boolalpha << checkMobileyeObstacleStop(nowtime) << "," << fabs(cmd_velocity) << ">" << current_velocity << "," << current_velocity + setting_.acceptable_velocity_variation << "<" << setting_.velocity_limit << "," << stopper_distance_.distance << "," << accel_mode_avoidance_distance << "," << in_accel_mode_;
			pub_tmp_.publish(str_pub.str());*/

			std::cout << "velocity hikaku : " << cmd_velocity << "," << current_velocity << std::endl;
			std::cout << "flag : " << (int)checkMobileyeObstacleStop(nowtime) << "," << stopper_distance_.distance << "," << in_accel_mode_ << "," << std::endl;
			//減速判定
			if(fabs(cmd_velocity) < current_velocity - setting_.acceptable_velocity_variation
			         && fabs(cmd_velocity) > 0.0 || (stopper_distance_.distance>=0 && stopper_distance_.distance <=current_velocity)
					 && in_brake_mode_ == true)
			{
				std::cout << "stroke brake" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_BRAKE_);
			}
			//停止線判定
			else if (stopper_distance_.distance >= 0 && stopper_distance_.distance < accel_avoidance_distance_min_ && in_brake_mode_ == true)
			{
				std::cout << "stroke distance" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_BRAKE_);
			}
			else if(stopper_distance_.send_process == autoware_msgs::StopperDistance::OBSTACLE && car_cruise_status_.accel_release == true)
			{
				std::cout << "stroke distance" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_ACCEL_RELEASE);
			}
			else if (checkMobileyeObstacleStop(nowtime) == false
					&& fabs(cmd_velocity) > current_velocity + setting_.acceptable_velocity_variation
			        && current_velocity < setting_.velocity_limit
			        && (stopper_distance_.distance<0 || stopper_distance_.distance>accel_mode_avoidance_distance)
					&& in_accel_mode_ == true)
			{
				std::cout << " stroke drive" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_ACCEL_);
			}
			//停止判定
			/*else if(cmd_velocity == 0.0 && current_velocity > 0.0)//VELOCITY_ZERO_VALUE_/100.0)
			{std::cout << "stroke stop" << std::endl;
				if(current_velocity < setting_.velocity_stop_th)
				{
					//new_stroke = _stopping_control(current_velocity);
					//pid_params.set_stop_stroke_prev(new_stroke);
					pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_STOP_);
				}
				else
				{
					//new_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity);
					pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_BRAKE_);
				}
			}*/
			else if(current_velocity > setting_.velocity_limit)
			{
				std::cout << "stroke over limit" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_KEEP_);
			}
			else {
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_KEEP_);
			}

			//td_msgs::String routine;
			/*switch(pid_params.get_stroke_state_mode_())
			{
			case PID_params::STROKE_STATE_MODE_ACCEL_:
				new_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity);//, &stroke_speed);
				routine_.data = "accel";
				pub_stroke_routine_.publish(routine_);
				break;
			case PID_params::STROKE_STATE_MODE_BRAKE_:
				new_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity, acceleration);//, &stroke_speed);
				

				std_msgs::Float64 brake_i;
				brake_i.data = e_i_val_;
				pub_brake_i_.publish(brake_i);
				new_stroke = _keep_control();//pid_params.get_stroke_prev();
				routine_.data = "keep";
				pub_stroke_routine_.publish(routine_);
				break;
			}*/

			switch(pid_params.get_stroke_state_mode_())
			{
			case PID_params::STROKE_STATE_MODE_ACCEL_:
				new_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity, routine_.data);//, &stroke_speed);
				pub_stroke_routine_.publish(routine_);
				break;
			case PID_params::STROKE_STATE_MODE_ACCEL_RELEASE:
				new_stroke = accel_release();
				break;
			case PID_params::STROKE_STATE_MODE_BRAKE_:
				std_msgs::Float64 brake_i;
				brake_i.data = e_i_val_;
				pub_brake_i_.publish(brake_i);
				new_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity, acceleration, routine_.data);//, &stroke_speed);
				/*if(stopper_distance_ >= 0 && stopper_distance_ <= 1.5 &&
					new_stroke > pid_params.get_stroke_prev())
						new_stroke = pid_params.get_stroke_prev();*/
				pub_stroke_routine_.publish(routine_);
				break;
			/*case PID_params::STROKE_STATE_MODE_STOP_:
				new_stroke = _stopping_control(current_velocity);
				pid_params.set_stop_stroke_prev(new_stroke);
				break;*/
			case PID_params::STROKE_STATE_MODE_KEEP_:
				new_stroke = _keep_control();//pid_params.get_stroke_prev();
				routine_.data = "keep";
				pub_stroke_routine_.publish(routine_);
				break;
			}
			//ブレーキを離す場合、徐々に離す
			/*if(pid_params.get_stroke_prev() < 0 && new_stroke >= 0)
			{
				double tmp = pid_params.get_stroke_prev() + stroke_speed;
				if(tmp < new_stroke) new_stroke = tmp;
			}
			//ブレーキをゆっくり踏む
			if(pid_params.get_stroke_prev() < 0.0 && pid_params.get_stroke_prev() < new_stroke)
			{
				double tmp = pid_params.get_stroke_prev() - stroke_speed;
				if(tmp > new_stroke) new_stroke = tmp;
				if(new_stroke < setting_.pedal_stroke_min) new_stroke = setting_.pedal_stroke_min;
			}
			pid_params.set_stroke_prev(new_stroke);*/

			//AUTOモードじゃない場合、stroke値0をcanに送る
			if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO ||
			    can_receive_503_.clutch == false)
			{
				pid_params.clear_diff_velocity();
				pid_params.clear_diff_acceleration();
				pid_params.clear_diff_distance();
				pid_params.set_stop_stroke_prev(0);
				pid_params.set_stroke_prev(0);
				short drive_val = 0;
				unsigned char *drive_point = (unsigned char*)&drive_val;
				buf[4] = drive_point[1];  buf[5] = drive_point[0];
				std::cout << "manual_mode" << std::endl;
				return;
			}

			short input_stroke = (short)new_stroke;
			if(input_drive_mode_ == true) input_stroke = input_drive_;
			unsigned char *drive_point = (unsigned char*)&input_stroke;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];
		}
	}

	void bufset_car_control(unsigned char *buf, double current_velocity)
	{
		buf[6] = buf[7] = 0;

		//２０２０年４月２１日番
		if(emergency_stop_ == 0x2) {buf[6] |= 0x80;  emergency_stop_ = 0;}
		else if(emergency_stop_ == 0x1) {buf[6] |= 0x40;  emergency_stop_ = 0;}
		//２０２１年１月０３日番
		//buf[6] |= 0x40;

		if(drive_clutch_ == false)
		{
			buf[6] |= 0x20;
			ros::Time time = ros::Time::now();
			if(time > drive_clutch_timer_) drive_clutch_ = true;
			else drive_clutch_ = false;
		}
		else if(interface_lock_ == true) buf[6] |= 0x20;
		//if(use_safety_localizer_ == false) buf[6] |= 0x10;
		//else
		if(steer_clutch_ == false)
		{
			buf[6] |= 0x10;
			ros::Time time = ros::Time::now();
			if(time > steer_clutch_timer_) steer_clutch_ = true;
			else steer_clutch_ = false;
		}
		else if(interface_lock_ == true) buf[6] |= 0x10;
		if(automatic_door_ != 0x0)
		{
			if(automatic_door_ == 0x2) {buf[6] |= 0x08;}
			else if(automatic_door_ == 0x1) {buf[6] |= 0x04;}
			ros::Time time = ros::Time::now();
			if(time > automatic_door_time_)  automatic_door_ = 0x0;
		}
		if(blinker_right_ == true)
		{
			buf[6] |= 0x02; //blinker_right_ = false;
			ros::Time time = ros::Time::now();
			//if(time > blinker_right_time_)  blinker_right_ = false;
		}
		else if(blinker_left_ == true)
		{
			buf[6] |= 0x01;
			ros::Time time = ros::Time::now();
			//if(time > blinker_left_time_)  blinker_left_ = false;
		}
		else if(blinker_stop_ == true)
		{
			buf[7] |= 0x40;
			ros::Time time = ros::Time::now();
			//if(time > blinker_stop_time_)  blinker_stop_ = false;
		}

		//if(light_high_ == true) buf[7] |= 0x20;
		//buf[7] |= 0x20;
		/*std_msgs::String str_tmp;
		str_tmp.data = fail_safe_flag_;
		pub_tmp_.publish(str_tmp);*/
		if((err_distance_ || err_localizer_time_ || err_localizer_stat_ || err_gnss_dev_ || !read_local_waypoints_) && use_error_check_)
		{
			buf[7] |= 0x20;
			buf[7] |= 0x10;
			//buf[6] |= 0x08;
			//buf[7] |= 0x08;
			//buf[6] |= 0x04;
			//buf[6] |= 0x20;
			//buf[6] |= 0x10;
			//buf[6] |= 0x40;
		}
		if(fail_safe_flag_ != "NONE") buf[6] |= 0x40;

		/*if (shift_auto_ == true)
		{
			buf[7] |= 0x08;
			switch (shift_position_)
			{
			case SHIFT_P:
				if(current_velocity > 0) buf[7] |= 0x00;
				break;
			case SHIFT_R:
				if(current_velocity > 0) buf[7] |= 0x01;
				break;
			case SHIFT_N:
				buf[7] |= 0x02;
				break;
			case SHIFT_D:
				buf[7] |= 0x03;
				break;
			case SHIFT_4:
				buf[7] |= 0x04;
				break;
			case SHIFT_L:
				buf[7] |= 0x05;
				break;
			}
		}*/
	}

	//クラスのメンバ関数をスレッド化するために必要
	static void* launchCanSendThread(void *pParam)
	{
       	reinterpret_cast<kvaser_can_sender*>(pParam)->can_send();
		pthread_exit(NULL);
    }

	//現在のlocalizerの更新が一定時間ない場合はエラー
	void localizerTimeCheck(ros::Time nowtime)
	{
		ros::Duration localizer_time_diff = nowtime - localizer_timer_;
		double localizer_time_dt = localizer_time_diff.sec + localizer_time_diff.nsec * 1E-9;
		if(use_error_check_ && localizer_time_dt > LOCALIZER_UPDATE_CHECK_TIME)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "localizer update time over : " << localizer_time_dt << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "localizer update time over : " << localizer_time_dt;
			publishStatus(safety_error_message.str());
			err_localizer_time_ = true;
			fail_safe_flag_ = "LocalizerTimeCheck";
		}
		else err_localizer_time_ = false;
	}
public:
	kvaser_can_sender(ros::NodeHandle nh, ros::NodeHandle p_nh)
	    : nh_(nh)
	    , private_nh_(p_nh)
	    , flag_drive_mode_(true)
	    , flag_steer_mode_(true)
	    , input_drive_mode_(true)
	    , input_steer_mode_(true)
	    , input_steer_(0)
	    , input_drive_(0)
	    , drive_control_mode_(MODE_STROKE)
	    , pedal_(0)
	    , shift_auto_(false)
	    , shift_position_(0)
	    , emergency_stop_(false)
	    , light_high_(false)
	    , blinker_right_(false)
	    , blinker_left_(false)
	    , blinker_stop_(false)
		, blinker_param_sender_(false)
	    , automatic_door_(0)
	    , drive_clutch_(true)
	    , steer_clutch_(true)
	    , use_velocity_data_(USE_VELOCITY_TWIST)
	    , use_acceleration_data_(USE_ACCELERATION_IMU)
	    , acceleration1_twist_(0)
	    , acceleration2_twist_(0)
	    , jurk1_twist_(0)
	    , jurk2_twist_(0)
	    , angle_limit_over_(false)
	    , steer_correction_(1.0)
		, localizer_select_num_(1)
		, accel_avoidance_distance_min_(20)
		, stop_stroke_max_(220)
		, in_accel_mode_(true)
		, in_brake_mode_(true)
		, use_stopper_distance_(true)
		, interface_lock_(false)
		, ndt_warning_count_(0)
		, use_safety_localizer_(true)
		, cruse_velocity_(0)
		, temporary_fixed_velocity_(0)
		, log_folder_("")
		, stop_distance_over_sum_(0)
		, stop_distance_over_add_(10.0/100.0)
		, use_slow_accel_release_(true)
		, thread_can_send_run_flag_(false)
		, first_lock_release_flag_(true)
		, steer_override_value_(STEER_OVERRIDE_TH)
		, drive_override_value_(DRIVE_OVERRIDE_TH)
		, mpc_steer_gradually_change_distance_(0)
		, last_steer_override_value_(0)
		, local_way_max_vel_mps_(0)
		, brake_stopD_first_velocity_(0)
		, use_error_check_(true)
		, auto_log_write_(true)
		, fail_safe_flag_("NONE")
		, read_local_waypoints_(false)
		, log_write_buttom_(false)
		, log_write_501_(false)
		, accel_stroke_cap_mobileye_(500)
		, accel_stroke_cap_temporary_stopper_(500)
		, update_current_velocity_(false)
		, e_d_(0)
	{
		stopper_distance_.distance = -1;
		stopper_distance_.send_process = autoware_msgs::StopperDistance::UNKNOWN;

		int kvaser_channel;
		private_nh_.param<int>("kvaser_channel", kvaser_channel, 0);
		private_nh_.param<int>("use_velocity_data", use_velocity_data_, USE_VELOCITY_TWIST);
		private_nh_.param<int>("use_acceleration_data", use_acceleration_data_, USE_ACCELERATION_IMU);

		//ステア角とホイール角の変換係数をglobal paramから読み込む デフォルトはレインボー用
		nh_.param<double>("/vehicle_info/wheelrad_to_steering_can_value_left", wheelrad_to_steering_can_value_left_, 25009.6727514125);
		nh_.param<double>("/vehicle_info/wheelrad_to_steering_can_value_right", wheelrad_to_steering_can_value_right_, 26765.9140133745);
		
		can_receive_501_.emergency = true;
		can_receive_501_.blinker = false;
		canStatus res = kc.init(kvaser_channel, canBITRATE_500K);
		//if(res != canStatus::canOK) {std::cout << "open error" << std::endl;}

		pub_microbus_can_sender_status_ = nh_.advertise<autoware_can_msgs::MicroBusCanSenderStatus>("/microbus/can_sender_status", 1, true);//このノードの現在の状態
		pub_log_write_ = nh_.advertise<std_msgs::String>("/microbus/log_write", 1);//logのトピック出力
		pub_localizer_match_stat_ = nh_.advertise<autoware_msgs::LocalizerMatchStat>("/microbus/localizer_match_stat", 1);//localizerが正常かの判定
		pub_stroke_routine_ = nh_.advertise<std_msgs::String>("/microbus/stroke_routine", 1);//storkeの処理遷移(accel,brake,keep)の表示
		pub_vehicle_status_ = nh_.advertise<autoware_msgs::VehicleStatus>("/microbus/vehicle_status", 1);//vehicle_statusトピックのpublish mpcが使用
		pub_velocity_param_ = nh_.advertise<autoware_can_msgs::MicroBusCanVelocityParam>("/microbus/velocity_param", 1);//速度に関する情報 加速度や過加速度もある
		pub_brake_i_ = nh_.advertise<std_msgs::Float64>("/microbus/brake_i", 1);//brake処理でのPID積分項の表示(確認用)
		pub_log_write_flag_ = nh_.advertise<std_msgs::Bool>("/microbus/log_write_flag", 1, true);//現在logを出力しているか？
		pub_acc_ = nh_.advertise<std_msgs::Float64>("/microbus/acc", 1);//加速度表示
		pub_tmp_ = nh_.advertise<std_msgs::String>("/microbus/tmp", 1);//即席表示用

		pub_acceleration_main = nh_.advertise<std_msgs::Float64>("/microbus/acceleration_main", 1);//brake処理でのPID積分項の表示(確認用)
		pub_acceleration_ideal = nh_.advertise<std_msgs::Float64>("/microbus/acceleration_ideal", 1);//brake処理でのPID積分項の表示(確認用)
		pub_list_acceleration = nh_.advertise<std_msgs::String>("/microbus/acceleration_list", 1);//即席表示用

		sub_microbus_drive_mode_ = nh_.subscribe("/microbus/drive_mode_send", 10, &kvaser_can_sender::callbackDModeSend, this);//driveのautoとmanualの切り替え
		sub_microbus_steer_mode_ = nh_.subscribe("/microbus/steer_mode_send", 10, &kvaser_can_sender::callbackSModeSend, this);//steerのautoとmanualの切り替え
		sub_twist_cmd_ = nh_.subscribe("/vehicle_cmd", 10, &kvaser_can_sender::callbackTwistCmd, this);//autowareからのコマンド指令
		sub_microbus_can_100_string_ = nh_.subscribe("/microbus/can_receive100_string", 10, &kvaser_can_sender::callbackMicrobusCan100String, this);
		sub_microbus_can_501_ = nh_.subscribe("/microbus/can_receive501", 10, &kvaser_can_sender::callbackMicrobusCan501, this);//canから送られてくる状態返答 ID 0x501
		sub_microbus_can_502_ = nh_.subscribe("/microbus/can_receive502", 10, &kvaser_can_sender::callbackMicrobusCan502, this);//canから送られてくる状態返答 ID 0x502
		sub_microbus_can_503_ = nh_.subscribe("/microbus/can_receive503", 10, &kvaser_can_sender::callbackMicrobusCan503, this);//canから送られてくる状態返答 ID 0x503
		sub_first_lock_release_ = nh_.subscribe("/microbus/first_lock_release", 10, &kvaser_can_sender::callbackFirstLockRelease, this);//ボードの立ち上げ時のlockを解除
		sub_input_steer_flag_ = nh_.subscribe("/microbus/input_steer_flag", 10, &kvaser_can_sender::callbackInputSteerFlag, this);//steer指令値外部入力モード
		sub_input_steer_value_ = nh_.subscribe("/microbus/input_steer_value", 10, &kvaser_can_sender::callbackInputSteerValue, this);//steer指令値を外部から入力
		sub_input_drive_flag_ = nh_.subscribe("/microbus/input_drive_flag", 10, &kvaser_can_sender::callbackInputDriveFlag, this);//drive指令値外部入力モード
		sub_input_drive_value_ = nh_.subscribe("/microbus/input_drive_value", 10, &kvaser_can_sender::callbackInputDriveValue, this);//drive指令値を外部から入力
		sub_stroke_mode_ = nh_.subscribe("/microbus/set_stroke_mode", 10, &kvaser_can_sender::callbackStrokeMode, this);//driveをstroke modeにする
		sub_velocity_mode_ = nh_.subscribe("/microbus/set_velocity_mode", 10, &kvaser_can_sender::callbackVelocityMode, this);//driveをvelocity modeにする
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 10, &kvaser_can_sender::callbackWaypointParam, this);//waypoint情報の取得
		sub_waypoints_ = nh_.subscribe("/final_waypoints", 10, &kvaser_can_sender::callbackWaypoints, this);//final_waypoints(確定local_waypoints)を取得
		sub_config_microbus_can_ = nh_.subscribe("/config/microbus_can", 10, &kvaser_can_sender::callbackConfigMicroBusCan, this);//このノードのconfig設定
		sub_config_localizer_switch_ = nh_.subscribe("/config/localizer_switch", 10, &kvaser_can_sender::callbackConfigLocalizerSwitch, this);//localizer_switchのconfig設定
		sub_shift_auto_ = nh_.subscribe("/microbus/shift_auto", 10, &kvaser_can_sender::callbackShiftAuto, this);//shiftのauto mode
		sub_shift_position_ = nh_.subscribe("/microbus/shift_position", 10, &kvaser_can_sender::callbackShiftPosition, this);//shiftのチェンジ
		sub_emergency_stop_ = nh_.subscribe("/microbus/emergency_stop", 10, &kvaser_can_sender::callbackEmergencyStop, this);//緊急停止指令
		sub_light_high_ = nh_.subscribe("/microbus/light_high", 10, &kvaser_can_sender::callbackLightHigh, this);//ハイライトをオン
		sub_blinker_right_ = nh_.subscribe("/microbus/blinker_right", 10, &kvaser_can_sender::callbackBlinkerRight, this);//左ウィンカー
		sub_blinker_left_ = nh_.subscribe("/microbus/blinker_left", 10, &kvaser_can_sender::callbackBlinkerLeft, this);//右ウィンカー
		sub_blinker_stop_ = nh_.subscribe("/microbus/blinker_stop", 10, &kvaser_can_sender::callbackBlinkerStop, this);//ウィンカー停止
		sub_automatic_door_ = nh_.subscribe("/microbus/automatic_door", 10, &kvaser_can_sender::callbackAutomaticDoor, this);//自動ドアのauto mode
		sub_drive_clutch_ = nh_.subscribe("/microbus/drive_clutch", 10, &kvaser_can_sender::callbackDriveClutch, this);//driveクラッチのON,OFF 2021/1/15ではOFFのみ可能
		sub_steer_clutch_ = nh_.subscribe("/microbus/steer_clutch", 10, &kvaser_can_sender::callbackSteerClutch, this);//steerクラッチのON,OFF 2021/1/15ではOFFのみ可能
		sub_econtrol_ = nh_.subscribe("/econtrol", 10, &kvaser_can_sender::callbackEControl, this);//改変velocity_setからpublishされる停止情報
		sub_stopper_distance_ = nh_.subscribe("/stopper_distance", 10, &kvaser_can_sender::callbackStopperDistance, this);
		sub_lidar_detector_objects_ = nh_.subscribe("/detection/lidar_detector/objects", 10, &kvaser_can_sender::callbackLidarDetectorObjects, this);
		sub_imu_ = nh_.subscribe("/gnss_imu", 10, &kvaser_can_sender::callbackImu, this);
		sub_gnss_standard_deviation_ = nh_.subscribe("/gnss_standard_deviation", 10, &kvaser_can_sender::callbackGnssStandardDeviation, this);
		sub_gnss_standard_deviation_sub_ = nh_.subscribe("/gnss_standard_deviation_sub", 10, &kvaser_can_sender::callbackGnssStandardDeviationSub, this);
		sub_gnss_sats_list_ = nh_.subscribe("/gnss_sats_list", 10, &kvaser_can_sender::callbackGnssSatsList, this);
		sub_gnss_imu_status_ = nh_.subscribe("/gnss_imu_status", 10, &kvaser_can_sender::callbackGnssImuStatus, this);
		sub_ndt_stat_string = nh_.subscribe("/ndt_monitor/ndt_status", 10, &kvaser_can_sender::callbackNdtStatString, this);
		sub_ndt_stat_ = nh_.subscribe("/ndt_stat", 10, &kvaser_can_sender::callbackNdtStat, this);
		sub_ndt_reliability_ = nh_.subscribe("/ndt_reliability", 10, &kvaser_can_sender::callbackNdtReliability, this);
		sub_gnss_stat_ = nh_.subscribe("/gnss_rtk_stat", 10, &kvaser_can_sender::callbackGnssStat, this);
		sub_ndt_pose_ = nh_.subscribe("/ndt_pose", 10, &kvaser_can_sender::callbackNdtPose, this);
		sub_gnss_pose_ = nh_.subscribe("/RTK_gnss_pose", 10, &kvaser_can_sender::callbackGnssPose, this);
		sub_antenna_pose_ = nh_.subscribe("/gnss_pose", 10, &kvaser_can_sender::callbackAntennaPose, this);
		sub_antenna_pose_sub_ = nh_.subscribe("/gnss_pose_sub", 10, &kvaser_can_sender::callbackAntennaPoseSub, this);
		sub_ekf_pose_ = nh_.subscribe("/ekf_pose", 10, &kvaser_can_sender::callbackEkfPose, this);
		sub_ekf_covariance_ = nh_.subscribe("/ekf_pose_with_covariance", 10, &kvaser_can_sender::callbackEkfCovariance, this);
		sub_difference_to_waypoint_distance_ = nh_.subscribe("/difference_to_waypoint_distance", 10, &kvaser_can_sender::callbackDifferenceToWaypointDistance, this);
		sub_difference_to_waypoint_distance_ndt_ = nh_.subscribe("/difference_to_waypoint_distance_ndt", 10, &kvaser_can_sender::callbackDifferenceToWaypointDistanceNdt, this);
		sub_difference_to_waypoint_distance_gnss_ = nh_.subscribe("/difference_to_waypoint_distance_gnss", 10, &kvaser_can_sender::callbackDifferenceToWaypointDistanceGnss, this);
		sub_difference_to_waypoint_distance_ekf_ = nh_.subscribe("/difference_to_waypoint_distance_ekf", 10, &kvaser_can_sender::callbackDifferenceToWaypointDistanceEkf, this);
		sub_localizer_select_num_ = nh_.subscribe("/localizer_select_num", 10, &kvaser_can_sender::callbackLocalizerSelectNum, this);                
		sub_interface_lock_ = nh_.subscribe("/microbus/interface_lock", 10, &kvaser_can_sender::callbackInterfaceLock, this);
		sub_use_safety_localizer_ = nh_.subscribe("/microbus/use_safety_localizer", 10, &kvaser_can_sender::callbackUseSafetyLocalizer, this);
		sub_config_current_velocity_conversion_ = nh_.subscribe("/config/current_velocity_conversion", 10, &kvaser_can_sender::callbackCurrentVelocityConversion, this);
		sub_cruse_velocity_ = nh_.subscribe("/cruse_velocity", 10, &kvaser_can_sender::callbackCruseVelocity, this);
		sub_mobileye_frame_ = nh.subscribe("/can_tx", 10 , &kvaser_can_sender::callbackMobileyeCan, this);
		sub_mobileye_obstacle_data_ = nh.subscribe("/use_mobileye_obstacle", 10 , &kvaser_can_sender::callbackMobileyeObstacleData, this);
		sub_front_mobileye_car_ = nh.subscribe("/mobileye_tracker/front_mobileye", 10 , &kvaser_can_sender::callbackFrontMobileyeCar, this);
		sub_temporary_fixed_velocity_ = nh.subscribe("/temporary_fixed_velocity", 10 , &kvaser_can_sender::callbackTemporaryFixedVelocity, this);
		sub_gnss_time_ = nh.subscribe("/gnss_time", 10 , &kvaser_can_sender::callbackGnssTime, this);
		sub_log_write_ = nh.subscribe("/microbus/log_on", 10 , &kvaser_can_sender::callbackLogWrite, this);
		sub_log_folder_ = nh.subscribe("/microbus/log_folder", 10 , &kvaser_can_sender::callbackLogFolder, this);
		sub_waypoints_file_name_ = nh.subscribe("/waypoints_file_name", 10 , &kvaser_can_sender::callbackWaypointFileName, this);
		sub_cruse_error_ = nh.subscribe("/cruse_error", 10 , &kvaser_can_sender::callbackCruseError, this);
		sub_steer_override_ = nh.subscribe("/microbus/steer_override", 10 , &kvaser_can_sender::callbackSteerOverride, this);
		sub_drive_override_ = nh.subscribe("/microbus/drive_override", 10 , &kvaser_can_sender::callbackDriveOverride, this);
		sub_way_increase_distance_ = nh.subscribe("/way_increase_distance", 10 , &kvaser_can_sender::callbackWayIncreaseDistance, this);
		sub_nmea_device_status_ = nh.subscribe("/nmea_device_status", 10 , &kvaser_can_sender::callbacNmeaDeviceStatus, this);
		sub_error_check_ = nh.subscribe("/microbus/error_check", 10 , &kvaser_can_sender::callbackErrorCheck, this);
		sub_shhv_ = nh.subscribe("/microbus/shhv", 10 , &kvaser_can_sender::callbackSHHV, this);
		sub_sppm_ = nh.subscribe("/microbus/sppm", 10 , &kvaser_can_sender::callbackSPPM, this);
		sub_auto_log_write_ = nh.subscribe("/microbus/auto_log_write", 10 , &kvaser_can_sender::callbackAutoLogWrite, this);
		sub_safety_waypoints_ = nh.subscribe("/safety_waypoints", 10 , &kvaser_can_sender::callbackSafetyWaypoints, this);
		sub_receiver_steer_correction_ = nh.subscribe("/microbus/receiver_steer_correction", 10 , &kvaser_can_sender::callbackReceiverSteerCorrection, this);
		sub_record_topic_list_ = nh.subscribe("/record_topic_list", 10 , &kvaser_can_sender::callbackRecordTopicList, this);
		sub_accel_stroke_cap_mobileye_ = nh.subscribe("/mobileye_tracker/accel_stroke_cap", 10 , &kvaser_can_sender::callbackAccelStrokeCapMobileye, this);
		sub_accel_stroke_cap_temporary_stopper_ = nh.subscribe("/temporary_stopper/accel_stroke_cap", 10 , &kvaser_can_sender::callbackAccelStrokeCapTemporaryStopper, this);
		sub_car_cruise_status_ = nh.subscribe("/car_cruise_status", 10 , &kvaser_can_sender::callbackCarCruiseStatus, this);

		//sub_interface_config_ = nh_.subscribe("/config/microbus_interface", 10, &kvaser_can_sender::callbackConfigInterface, this);

		sub_current_pose_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/current_pose", 10);
		sub_current_velocity_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, "/current_velocity", 10);
		sync_twist_pose_ = new message_filters::Synchronizer<TwistPoseSync>(TwistPoseSync(SYNC_FRAMES), *sub_current_velocity_, *sub_current_pose_);
		sync_twist_pose_->registerCallback(boost::bind(&kvaser_can_sender::TwistPoseCallback, this, _1, _2));

		std::string safety_error_message = "";
		publishStatus(safety_error_message);

		waypoint_param_.blinker = 0;
		automatic_door_time_ = blinker_right_time_ = blinker_left_time_ =
		        blinker_stop_time_ = can_send_time_ = localizer_timer_ = ros::Time::now();
		gnss_stat_error_time_ = ros::Time(0);

		ros::Time nowtime = ros::Time::now();
		pid_params.init(0.0);
		mobileye_obstacle_data_.header.stamp = ros::Time(0);
		waypoint_param_.id = -1;
		waypoint_param_.steer_override = STEER_OVERRIDE_TH;
		current_velocity_.header.stamp = nowtime;
		current_velocity_.twist.linear.x = 0;
		current_velocity_.twist.angular.z = 0;
		//ボード送信をスレッド化
		if(pthread_create(&thread_can_send_, nullptr, kvaser_can_sender::launchCanSendThread, this) == 0)
		{
			if(pthread_detach(thread_can_send_) == 0) thread_can_send_run_flag_ = true;
		}
	}

	bool is_can_send_thread()
	{
		return thread_can_send_run_flag_;
	}

	void end_can_send()
	{
		thread_can_send_run_flag_ = false;
	}

	~kvaser_can_sender()
	{
		pthread_cancel(thread_can_send_);
	    pthread_join(thread_can_send_, NULL);

		delete sync_twist_pose_;
		delete sub_current_pose_;
		delete sub_current_velocity_;
	}

	const bool isOpen() {return kc.isOpen();}

	void can_send()
	{
		ros::Rate rate(100);
		while(thread_can_send_run_flag_ == true)
		{
			ros::Time nowtime = ros::Time::now();
			ros::Duration time_diff = nowtime - can_send_time_;
			double t_diff = time_diff.sec + time_diff.nsec * 1E-9;

			writeLog(nowtime);

			if(first_lock_release_flag_ == true)
			{
				ros::Rate eme_rate(10);
				std::cout << "sub Emergency" << std::endl;
				//char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};
				memset(id100_send_, 0, SEND_DATA_SIZE);
				id100_send_[0] = 0x55;
				kc.write(0x100, id100_send_, SEND_DATA_SIZE);
				eme_rate.sleep();
				id100_send_[0] = 0x00;
				kc.write(0x100, id100_send_, SEND_DATA_SIZE);
				first_lock_release_flag_ = false;
			}
			else
			{
				can_send_time_ = nowtime;
				//現在のlocalizerの更新が一定時間ない場合はエラー
				localizerTimeCheck(nowtime);

				//ndtとgnssのチェック
				NdtGnssCheck(nowtime);

				double current_velocity = 0;// = can_receive_502_.velocity_average / 100.0;

				switch(use_velocity_data_)
				{
				case USE_VELOCITY_CAN:
					current_velocity = can_receive_502_.velocity_average / 100.0;
					break;
				case USE_VELOCITY_TWIST:
					current_velocity = current_velocity_.twist.linear.x * 3.6;
					break;
				default:
					current_velocity = current_velocity_.twist.linear.x * 3.6;
					break;
				}

				double acceleration = 0;
				use_acceleration_data_ = USE_ACCELERATION_TWIST2;
				switch(use_acceleration_data_)
				{
				case USE_ACCELERATION_TWIST1:
					acceleration = acceleration1_twist_;
					break;
				case USE_ACCELERATION_TWIST2:
					acceleration = acceleration2_twist_;
					break;
				case USE_ACCELERATION_IMU:
					/*double ax = imu_.linear_acceleration.x;
					double ay = imu_.linear_acceleration.y;
					double az = imu_.linear_acceleration.z;
					acceleration = sqrt(ax*ax + ay*ay + az*az);*/
					acceleration = imu_.linear_acceleration.x;
					break;
				default:
					break;
				}

				//unsigned char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};
				memset(id100_send_, 0, SEND_DATA_SIZE);
				bufset_mode(id100_send_);
				bufset_steer(id100_send_, nowtime);
				bufset_drive(id100_send_, current_velocity, acceleration, 2.0);
				bufset_car_control(id100_send_, current_velocity);

				autoware_msgs::VehicleStatus status;
				status.header.stamp = ros::Time::now();
				status.drivemode = (id100_send_[0] & 0x0B != 0x0B) ? autoware_msgs::VehicleStatus::MODE_AUTO : autoware_msgs::VehicleStatus::MODE_MANUAL;
				status.steeringmode = (id100_send_[0] & 0xA0 != 0xA0) ? autoware_msgs::VehicleStatus::MODE_AUTO : autoware_msgs::VehicleStatus::MODE_MANUAL;
				status.current_gear.gear = autoware_msgs::Gear::NONE;
				status.lamp = 0;
				status.light = 0;
				status.speed = current_velocity_.twist.linear.x * 3.6;
				int16_t angle_val = (waypoint_param_.mpc_target_input == 0) ? can_receive_502_.angle_actual : target_steer_;
				/*int16_t angle_val;
				if(waypoint_param_.mpc_target_input == 0)
				{
					//steerの上書きからmpcへの変化をゆっくり行う
					//c1 = 0 でコマンドから計算した指令値がangle_actualに入る
					/*double c1 = (MPC_STEER_GRADUALLY_CHANGE_DISTANCE_INIT - mpc_steer_gradually_change_distance_) / MPC_STEER_GRADUALLY_CHANGE_DISTANCE_INIT;
					double c2 = 1 - c1;
					angle_val = c1 * can_receive_502_.angle_actual + c2 * last_steer_override_value_;

					std::stringstream str_pub;
					str_pub << c1 << "," << c2 << "," << mpc_steer_gradually_change_distance_;
					pub_tmp_.publish(str_pub.str());
				}
				else angle_val = target_steer_;*/
				if(can_receive_502_.angle_actual > 0) status.angle = angle_val / wheelrad_to_steering_can_value_left_;
				else status.angle = angle_val / wheelrad_to_steering_can_value_right_;
				pub_vehicle_status_.publish(status);

				kc.write(0x100, (char*)id100_send_, SEND_DATA_SIZE);
				rate.sleep();
			}
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_microbus_can_sender_stroke");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	kvaser_can_sender kcs(nh, private_nh);
	//if(kcs.isOpen() == false || kcs.is_can_send_thread() == false)
	if(kcs.is_can_send_thread() == false)
	{
		std::cerr << "error : open" << std::endl;
		return 0;
	}

	ros::spin();
	kcs.end_can_send();
	return 0;
}