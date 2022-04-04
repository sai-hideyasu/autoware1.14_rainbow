#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_config_msgs/ConfigMicroBusCan.h>
#include <autoware_config_msgs/ConfigTemporaryStopper.h>
#include <autoware_config_msgs/ConfigVelocitySet.h>
#include <autoware_config_msgs/ConfigLaneRule.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/StopperDistance.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/Gear.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_can_msgs/MicroBusCanSenderStatus.h>
#include <autoware_can_msgs/MicroBusCanVelocityParam.h>
#include <autoware_can_msgs/MicroBusPseudoParams.h>
#include <autoware_msgs/SteerOverride.h>
#include "microbus_params.h"

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
	double get_acclel_diff_sum_velocity() {return accel_diff_sum_velocity_;}
	double get_acclel_diff_sum_acceleration() {return accel_diff_sum_acceleration_;}
	double get_acclel_diff_sum_distance() {return accel_diff_sum_distance_;}
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

class MicrobusPseudoCanSender
{
private:
	//velcity params
	const short VELOCITY_ZERO_VALUE_ = 132;

	//use velocity topic
	const static int USE_VELOCITY_CAN = 0;
	const static int USE_VELOCITY_TWIST = 1;

	//stroke params
	const short PEDAL_VOLTAGE_CENTER_ = 1024;//1052;//計測値は1025;

	//shift_param
	const static unsigned char SHIFT_P = 0;
	const static unsigned char SHIFT_R = 1;
	const static unsigned char SHIFT_N = 2;
	const static unsigned char SHIFT_D = 3;
	const static unsigned char SHIFT_4 = 4;
	const static unsigned char SHIFT_L = 5;

	//other params
	const unsigned int SEND_DATA_SIZE = 8;//CAN通信のバッファサイズ

	//steer_overwirte
	const static double STEER_OVERWRIDE_TH = -100000;//sterrのactualを上書きする判定のしきい値

	ros::NodeHandle nh_, p_nh_;

	ros::Publisher pub_microbus_can_sender_status_, pub_stroke_process_, pub_stroke_routine_, pub_vehicle_status_;
	ros::Publisher pub_velocity_param_, pub_pseudo_params_, pub_tmp_;

	ros::Subscriber sub_vehicle_cmd_, sub_microbus_drive_mode_, sub_microbus_steer_mode_;
	ros::Subscriber sub_microbus_can_501_, sub_microbus_can_502_, sub_microbus_can_503_, sub_stopper_distance_;
	ros::Subscriber sub_config_, sub_config_velocity_set_, sub_config_temporary_stopper_, sub_config_lane_rule_;
	ros::Subscriber sub_emergency_stop_, sub_drive_clutch_, sub_steer_clutch_, sub_interface_lock_;
	ros::Subscriber sub_automatic_door_, sub_blinker_right_, sub_blinker_left_, sub_blinker_stop_;
	ros::Subscriber sub_waypoint_param_, sub_steer_override_;

	message_filters::Subscriber<geometry_msgs::TwistStamped> *sub_current_velocity_;
	message_filters::Subscriber<geometry_msgs::PoseStamped> *sub_current_pose_;
	message_filters::Synchronizer<TwistPoseSync> *sync_twist_pose_;

	autoware_config_msgs::ConfigMicroBusCan setting_;
	autoware_config_msgs::ConfigTemporaryStopper config_temporary_stopper_;
	autoware_config_msgs::ConfigVelocitySet config_velocity_set_;
	autoware_config_msgs::ConfigLaneRule config_lane_rule_;

	autoware_can_msgs::MicroBusCan501 can_receive_501_;
	autoware_can_msgs::MicroBusCan502 can_receive_502_;
	autoware_can_msgs::MicroBusCan503 can_receive_503_;
	autoware_msgs::VehicleCmd vehicle_cmd_;//autowareからの各種コマンド値
	geometry_msgs::PoseStamped current_pose_;
	geometry_msgs::TwistStamped current_velocity_;
	double cmd_steer_diff_; //vehicle_cmdのsteering_angleの前回との差分
	bool flag_drive_mode_, flag_steer_mode_;//Autoモードかのフラグ
	bool input_drive_mode_, input_steer_mode_;//Inputモードかのフラグ(Autoモードで手動入力を行うか)
	short input_steer_, input_drive_;//Inputモード時の入力値
	bool angle_limit_over_;//steerの角度エラーがでたのか？
	double accel_avoidance_distance_min_;//坂ではこの数字を小さくすると加速しなくなる
	autoware_msgs::StopperDistance stopper_distance_;//停止線情報
	bool in_accel_mode_, in_brake_mode_;//アクセルモードまたはブレーキモードを機能させるかのフラグ
	PID_params pid_params_;
	double stop_distance_over_add_, stop_distance_over_sum_;//停止線に到達した場合の踏み増し処理用変数
	double send_step_;//strokeのステップ(外部出力用、処理には使われない)
	std_msgs::String routine_;//ドライブでの処理プロセス(アクセルやブレーキ)の外部表示変数
	double acceleration1_twist_, acceleration2_twist_, jurk1_twist_, jurk2_twist_;
	std::vector<double> acceleration_vec1_, acceleration_vec2_;
	int stopper_distance_flag_;//停止線処理のどこを処理したかを示す(外部表示用)
	bool use_stopper_distance_;//一時停止情報を運用するかのフラグ
	int use_stopper_distance_val_for_brake_;//brakeルーチンでの停止線の有無
	double stopper_distance_fixed_velocity_for_brake_;//brakeルーチンでの停止線の速度
	int stopper_distance_send_process_for_brake_;//brakeルーチンでの停止線が送られたプロセスの種類
	unsigned int loop_counter_;//can_sendの処理カウンター
	unsigned char emergency_stop_;
	unsigned char drive_clutch_, steer_clutch_, automatic_door_, shift_position_;
	bool interface_lock_;
	bool blinker_right_, blinker_left_, blinker_stop_;
	bool light_high_, shift_auto_;
	autoware_msgs::WaypointParam waypoint_param_;
	autoware_can_msgs::MicroBusPseudoParams pseudo_params_;//疑似receiverノードに擬似データを送るためのメッセージ変数
	double brake_stroke_cap_;//brake_strokeの特殊キャップ値
	double steer_override_value_;//-100000以上の場合、steerのtargetをこの値に上書きする
	double target_steer_;//現在canに送信したsteer_targetの値

	ros::Time drive_clutch_timer_, steer_clutch_timer_, automatic_door_time_;
	ros::Time blinker_right_time_, blinker_left_time_, blinker_stop_time_;

	void publishStatus(std::string safety_error_message)
	{
		autoware_can_msgs::MicroBusCanSenderStatus msg;
		msg.header.stamp = ros::Time::now();
		msg.use_position_checker = setting_.use_position_checker;
		msg.use_input_steer = input_steer_mode_;
		msg.use_input_drive = input_drive_mode_;
		msg.use_velocity_topic = USE_VELOCITY_TWIST;
		//msg.position_check_stop = position_checker_.stop_flag;
		msg.angle_limit_over = angle_limit_over_;
		if(safety_error_message != "") msg.safety_error_message = safety_error_message;
		else msg.safety_error_message = "";
		std::cout << msg.safety_error_message << std::endl;
		pub_microbus_can_sender_status_.publish(msg);
	}

	void callbackConfigMicroBusCan(const autoware_config_msgs::ConfigMicroBusCan::ConstPtr &msg)
	{
		setting_ = *msg;
		brake_stroke_cap_ = setting_.pedal_stroke_min;
		std::string safety_error_message = "";
		publishStatus(safety_error_message);
	}

	void callbackConfigVelocitySet(const autoware_config_msgs::ConfigVelocitySet::ConstPtr &msg)
	{
		config_velocity_set_ = *msg;
	}

	void callbackConfigTemporaryStopper(const autoware_config_msgs::ConfigTemporaryStopper::ConstPtr &msg)
	{
		config_temporary_stopper_ = *msg;
	}

	void callbackConfigLaneRule(const autoware_config_msgs::ConfigLaneRule::ConstPtr &msg)
	{
		config_lane_rule_ = *msg;
	}

	void callbackMicrobusCan501(const autoware_can_msgs::MicroBusCan501::ConstPtr &msg)
	{
		std::cout << "sub can_501" << std::endl;

		/*if(msg->soot_combustion == true)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Denger! Soot combustion" << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "Denger! Soot combustion";
			publishStatus(safety_error_message.str());
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			can_send();
		}*/

		can_receive_501_ = *msg;
	}

	void callbackMicrobusCan502(const autoware_can_msgs::MicroBusCan502::ConstPtr &msg)
	{
		std::cout << "sub can_502" << std::endl;
		std::cout << "time : " << msg->header.stamp.sec << "," << msg->header.stamp.nsec << std::endl;
		std::cout << "deg : " << msg->angle_deg << "," << msg->angle_deg_acc << std::endl;
		if(msg->clutch==true && can_receive_502_.clutch==false)
		{
			system("aplay -D plughw:PCH /home/autoware/steer_clutch_on.wav &");
			input_steer_mode_ = false; //std::cout << "aaa" << std::endl;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}
		if(msg->clutch==false && can_receive_502_.clutch==true)
		{
			system("aplay -D plughw:PCH /home/autoware/steer_clutch_off.wav &");
			input_steer_mode_ = true;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}

		can_receive_502_ = *msg;
	}

	void callbackMicrobusCan503(const autoware_can_msgs::MicroBusCan503::ConstPtr &msg)
	{
		std::cout << "sub can_503" << std::endl;
		if(msg->clutch==true && can_receive_503_.clutch==false)
		{
			system("aplay -D plughw:PCH /home/autoware/drive_clutch_on.wav &");
			//drive_control_mode_ = MODE_VELOCITY;
			//shift_auto_ = true;
			input_drive_mode_ = false;
			//pid_params_.set_stroke_prev(can_receive_503_.pedal_displacement);
			pid_params_.set_stroke_prev(0);//ガタンの暫定対処
			pid_params_.set_stop_stroke_prev(0);//ガタンの暫定対処
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}
		if(msg->clutch==false && can_receive_503_.clutch==true)
		{
			system("aplay -D plughw:PCH /home/autoware/drive_clutch_off.wav &");
			//drive_control_mode_ = MODE_STROKE;
			//shift_auto_ = false;
			input_drive_mode_ = true;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}

		can_receive_503_ = *msg;
	}

	void callbackDModeSend(const std_msgs::Bool::ConstPtr &msg)//driveのautoモードチェンジ
	{
		std::string flag = (msg->data) ? "on" : "off";
		std::cout << "sub DMode : " << flag << std::endl;
		flag_drive_mode_ = msg->data;
	}

	void callbackSModeSend(const std_msgs::Bool::ConstPtr &msg)//steerのautoモードチェンジ
	{
		std::string flag = (msg->data) ? "on" : "off";
		std::cout << "sub SMode : " << flag << std::endl;
		flag_steer_mode_ = msg->data;
	}

	void callbackVehicleCmd(const autoware_msgs::VehicleCmd::ConstPtr &msg)
	{
		std::cout << "sub twist" << std::endl;

		ros::Duration rostime_diff = msg->header.stamp - vehicle_cmd_.header.stamp;
		double time_diff = rostime_diff.sec + rostime_diff.nsec * 1E-9;
		double ws_ave = (wheelrad_to_steering_can_value_left + wheelrad_to_steering_can_value_right) / 2.0;
		double deg = fabs(msg->ctrl_cmd.steering_angle - vehicle_cmd_.ctrl_cmd.steering_angle) * ws_ave * 720.0 / 15000.0;
		cmd_steer_diff_ = msg->ctrl_cmd.steering_angle - vehicle_cmd_.ctrl_cmd.steering_angle * ws_ave * 720.0 / 15000.0;
		double zisoku = msg->ctrl_cmd.linear_velocity * 3.6;

		vehicle_cmd_ = *msg;
	}

	void TwistPoseCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
						   const geometry_msgs::PoseStampedConstPtr &pose_msg)
	{
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

		current_velocity_ = *twist_msg;
		current_pose_ = *pose_msg;
		acceleration1_twist_ = acc;
		acceleration2_twist_ = acc2;
		jurk1_twist_ = jurk;
		jurk2_twist_ = jurk2;

		current_velocity_ = *twist_msg;
		current_pose_ = *pose_msg;
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

	void callbackDriveClutch(const std_msgs::Bool::ConstPtr &msg)
	{
		drive_clutch_ = msg->data;
		ros::Time nowtime = ros::Time::now();
		if(drive_clutch_ == true) brake_stroke_cap_ = 0;//クラッチがONになったらブレーキキャップを0にする（ガタン対策）
		drive_clutch_timer_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackSteerClutch(const std_msgs::Bool::ConstPtr &msg)
	{
		steer_clutch_ = msg->data;
		ros::Time nowtime = ros::Time::now();
		steer_clutch_timer_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
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

	void callbackInterfaceLock(const std_msgs::BoolConstPtr &msg)
	{
		interface_lock_ = msg->data;
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		//pedal_ = msg->microbus_pedal;

		/*if(msg->id != waypoint_param_.id)
		{
			double way_distance_ave = way_distance_sum_ / way_distance_count_;
			double way_distance_ave_ndt = way_distance_sum_ndt_ / way_distance_count_ndt_;
			double way_distance_ave_gnss = way_distance_sum_gnss_ / way_distance_count_gnss_;
			double way_distance_ave_ekf = way_distance_sum_ekf_ / way_distance_count_ekf_;
			geometry_msgs::Point way_pose_ave, way_pose_ave_ndt, way_pose_ave_gnss, way_pose_ave_ekf;
			way_pose_ave.x = way_pose_sum_.x / way_distance_count_; way_pose_ave.y = way_pose_sum_.y / way_distance_count_; way_pose_ave.z = way_pose_sum_.z / way_distance_count_;
			way_pose_ave_ndt.x = way_pose_sum_ndt_.x / way_distance_count_ndt_; way_pose_ave_ndt.y = way_pose_sum_ndt_.y / way_distance_count_ndt_; way_pose_ave_ndt.z = way_pose_sum_ndt_.z / way_distance_count_ndt_;
			way_pose_ave_gnss.x = way_pose_sum_gnss_.x / way_distance_count_gnss_; way_pose_ave_gnss.y = way_pose_sum_gnss_.y / way_distance_count_gnss_; way_pose_ave_gnss.z = way_pose_sum_gnss_.z / way_distance_count_gnss_;
			way_pose_ave_ekf.x = way_pose_sum_ekf_.x / way_distance_count_ekf_; way_pose_ave_ekf.y = way_pose_sum_ekf_.y / way_distance_count_ekf_; way_pose_ave_ekf.z = way_pose_sum_ekf_.z / way_distance_count_ekf_;

			way_distance_sum_ = 0;  way_distance_count_ = 0;
			way_distance_sum_ndt_ = 0;  way_distance_count_ndt_ = 0;
			way_distance_sum_gnss_ = 0;  way_distance_count_gnss_ = 0;
			way_distance_sum_ekf_ = 0;  way_distance_count_ekf_ = 0;
			way_pose_sum_.x = way_pose_sum_.y = way_pose_sum_.z = 0;
			way_pose_sum_ndt_.x = way_pose_sum_ndt_.y = way_pose_sum_ndt_.z = 0;
			way_pose_sum_gnss_.x = way_pose_sum_gnss_.y = way_pose_sum_gnss_.z = 0;
			way_pose_sum_ekf_.x = way_pose_sum_ekf_.y = way_pose_sum_ekf_.z = 0;

			std::stringstream str;
			str << "," << waypoint_param_.id << ",current," << way_distance_ave << ",ndt," << way_distance_ave_ndt;
			str << ",gnss," << way_distance_ave_gnss << ",ekf," << way_distance_ave_ekf;
			str << ",cur_x," << way_pose_ave.x << "cur_y," << way_pose_ave.y << ",cur_z," << way_pose_ave.z;
			str << ",ndt_x," << way_pose_ave_ndt.x << ",ndt_y," << way_pose_ave_ndt.y << ",ndt_z," << way_pose_ave_ndt.z;
			str << ",gnss_x," << way_pose_ave_gnss.x << ",gnss_y," << way_pose_ave_gnss.y << ",gnss_z," << way_pose_ave_gnss.z;
			str << ",ekf_x," << way_pose_ave_ekf.x << ",ekf_y," << way_pose_ave_ekf.y << ",ekf_z," << way_pose_ave_ekf.z;
			std_msgs::String pub;
			pub.data = str.str();
			pub_way_distance_ave_.publish(pub);
		}*/

		/*if(msg->localizer_check > 0)
		{
			config_localizer_switch_.localizer_check = msg->localizer_check;
		}*/

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

		/*if(can_receive_502_.clutch == false && can_receive_503_.clutch == false && blinker_param_sender_ == true)
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
		}*/

		if(msg->liesse.shift >= 0)
		{
			shift_position_ = msg->liesse.shift;
		}

		//if(msg->steer_correction > -1000)
		//{
		//	steer_correction_ = msg->steer_correction;
		//	if(steer_correction_ > 500 || steer_correction_ < -500) steer_correction_ = 0;
		//}
		/*if(msg->steer_correction > 0)
		{
			steer_correction_  = msg->steer_correction;
		}*/

		if(msg->accel_stroke_offset >= 0 && msg->accel_stroke_offset <= 300)
		{ 
			setting_.accel_stroke_offset = msg->accel_stroke_offset;
		}

		if(msg->accel_avoidance_distance_min >= 0 && msg->accel_avoidance_distance_min <= 100)
		{
			accel_avoidance_distance_min_ = msg->accel_avoidance_distance_min;
			//std::cout << "kkk accel_avoidance_distance_min:  " << accel_avoidance_distance_min_ << std::endl;
		}
		if(msg->stop_stroke_max <= -300 && msg->stop_stroke_max >= -500)
		{
			//setting_.stop_stroke_max = msg->stop_stroke_max;
			//std::cout << "kkk stop_stroke_min : " << setting_.stop_stroke_max << std::endl;
		}
		if(msg->accel_stroke_max >= 300 && msg->accel_stroke_max <= 850)
		{
			setting_.pedal_stroke_max = msg->accel_stroke_max;
		}

		if(msg->accel_stroke_step_max > 0) setting_.accel_stroke_step_max = msg->accel_stroke_step_max;
		if(msg->accel_stroke_step_min > 0) setting_.accel_stroke_step_min = msg->accel_stroke_step_min;
		if(msg->brake_stroke_step_max > 0) setting_.brake_stroke_step_max = msg->brake_stroke_step_max;
		if(msg->brake_stroke_step_min > 0) setting_.brake_stroke_step_min = msg->brake_stroke_step_min;

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
		
		if(msg->obstacle_deceleration >= 0) config_velocity_set_.deceleration_obstacle = msg->obstacle_deceleration;
		steer_override_value_ = msg->steer_override;

		waypoint_param_ = *msg;
	}

	void callbackSteerOverride(const autoware_msgs::SteerOverride::ConstPtr &msg)
	{
		steer_override_value_ = msg->steer_value;
	}

	void bufset_mode(unsigned char *buf)
	{
		unsigned char mode = 0;
		if(flag_drive_mode_ == true) mode |= 0xA0;
		if(flag_steer_mode_ == true) mode |= 0xA0;
		buf[0] = mode;  buf[1] = 0;
	}

	void bufset_steer(unsigned char *buf)
	{
		short steer_val;

		if(input_steer_mode_ == false)
		{
			if(steer_override_value_ > STEER_OVERWRIDE_TH)//steerのオーバーライドがON
			{
				steer_val = steer_override_value_;
				std::cout << "steer_override" << std::endl;
			}
			else
			{
				double wheel_ang = vehicle_cmd_.ctrl_cmd.steering_angle;
				double zisoku = vehicle_cmd_.ctrl_cmd.linear_velocity * 3.6;
				if(wheel_ang > 0)
				{
					steer_val = (wheel_ang * wheelrad_to_steering_can_value_left
									+ wheelrad_to_steering_can_value_left_intercept);// * steer_correction_;
				}
				else
				{
					steer_val = (wheel_ang * wheelrad_to_steering_can_value_right
									+ wheelrad_to_steering_can_value_right_intercept);// * steer_correction_;
				}
			}
			//std::cout << "steer_correction : " << steer_correction_ << std::endl;
		}
		else steer_val = input_steer_;

		if(can_receive_501_.steer_auto != autoware_can_msgs::MicroBusCan501::STEER_AUTO) steer_val = 0;

		unsigned char *steer_pointer = (unsigned char*)&steer_val;
		buf[2] = steer_pointer[1];  buf[3] = steer_pointer[0];

		pseudo_params_.angle_actual = steer_val;
		target_steer_ = steer_val;
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

	bool stopper_distance_accele_checker(double accel_mode_avoidance_distance, double current_velocity)
	{
		switch(stopper_distance_.send_process)
		{
			case autoware_msgs::StopperDistance::UNKNOWN:
				return true;
			case autoware_msgs::StopperDistance::TEMPORARY_STOPPER:
				if(stopper_distance_.distance < 0 || stopper_distance_.distance>accel_mode_avoidance_distance)//停止線に到達していない
					return true;
				else
				{
					if(stopper_distance_.distance == 0) return false;//停止点にいる場合は加速しない
					else if(stopper_distance_.fixed_velocity == 0) return false;//停止点速度が0の場合は加速しない
					else if(stopper_distance_.fixed_velocity < current_velocity) return false;//現在速度が停止線速度より速い場合は加速しない
					else return true;//加速する
				}
				
				/*if(stopper_distance_.distance<0 || stopper_distance_.distance>accel_mode_avoidance_distance ||
				   stopper_distance_.fixed_velocity != 0)
					return true;
				else return false;*/
			case autoware_msgs::StopperDistance::SIGNAL:
				if(stopper_distance_.distance<0 || stopper_distance_.distance>accel_mode_avoidance_distance)
					return true;
				else return false;
			case autoware_msgs::StopperDistance::OBSTACLE:
				if(stopper_distance_.distance<0 || stopper_distance_.distance>accel_mode_avoidance_distance)
					return true;
				else return false;
			default:
				return false;
		}
	}

	//下支え用
	double math_stroke_kagen_accle(double current_velocity)
	{
		const double minvel = 10;
		const double maxvel = 20;
		const double minsrk = 0;
		const double maxsrk = 200;
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

	//下支え用
	double math_stroke_kagen_brake(double current_velocity)
	{
		const double minvel = 10;
		const double maxvel = 20;
		const double minsrk = 0;
		const double maxsrk = 200;
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

	double _accel_stroke_pid_control(double current_velocity, double cmd_velocity)
	{
		stop_distance_over_sum_ = 0;

		//cmd_velocityとcurrent_velocityの差が小さい場合、stepを小さくする
		double accle_stroke_step = setting_.accel_stroke_step_max;//3;
		double vel_sa = cmd_velocity - current_velocity;
		double accel_stroke_adjust_th = (cmd_velocity + current_velocity)/2.0 * (setting_.accel_stroke_adjust_th/ 100.0);
		if(vel_sa < accel_stroke_adjust_th)
		//if(vel_sa < setting_.accel_stroke_adjust_th)
		{
			//accle_stroke_step -= (setting_.accel_stroke_adjust_th-vel_sa)*setting_.accel_stroke_step_max/setting_.accel_stroke_adjust_th;
			accle_stroke_step -= (setting_.accel_stroke_adjust_th-vel_sa)*setting_.accel_stroke_step_max/accel_stroke_adjust_th;
			if(accle_stroke_step < 0.5) accle_stroke_step = setting_.accel_stroke_step_min;
		}

		//ブレーキからアクセルに変わった場合、Iの積算値をリセット
		//double stroke = PEDAL_VOLTAGE_CENTER_ - can_receive_503_.pedal_voltage;
		double stroke = can_receive_503_.pedal_displacement;
		std::cout << "voltage stroke : " << stroke << std::endl;
		std::cout << "voltage center : " << PEDAL_VOLTAGE_CENTER_ << std::endl;
		//std::cout << "voltage        : " << can_receive_503_.pedal_voltage << std::endl;
		std::cout << "brake offset   : " << setting_.brake_stroke_offset << std::endl;

		if(pid_params_.get_stop_stroke_prev() > 0)
		{
			double ret = pid_params_.get_stop_stroke_prev();
			ret -= accle_stroke_step;
			if(ret < 0) ret = 0;
			pid_params_.set_stop_stroke_prev(ret);
			return -ret;
		}
		pid_params_.set_brake_e_prev_velocity(0);
		pid_params_.set_brake_e_prev_acceleration(0);
		pid_params_.set_stroke_prev(0);

		//P
		double e = cmd_velocity - current_velocity;

		//I
		double e_i;
		pid_params_.plus_accel_diff_sum_velocity(e);
		if (pid_params_.get_acclel_diff_sum_velocity() > setting_.accel_max_i)
			e_i = setting_.accel_max_i;
		else
			e_i = pid_params_.get_acclel_diff_sum_velocity();

		//D
		double e_d = e - pid_params_.get_accel_e_prev_velocity();

		double target_accel_stroke = setting_.k_accel_p_velocity * e +
			   setting_.k_accel_i_velocity * e_i +
			   setting_.k_accel_d_velocity * e_d;

		pid_params_.set_accel_e_prev_velocity(e);

		double ret = target_accel_stroke;
		if(ret > setting_.pedal_stroke_max)
			ret = setting_.pedal_stroke_max;
		else if (ret < setting_.pedal_stroke_center)
			ret = setting_.pedal_stroke_center;
		if(ret < setting_.accel_stroke_offset) ret = setting_.accel_stroke_offset;

		if(pid_params_.get_stroke_prev() < 0 && ret >= 0)
		{
			double tmp = pid_params_.get_stroke_prev() + accle_stroke_step;
			if(tmp < ret) ret = tmp;
		}

		std_msgs::String str_stroke_process;
		double stroke_kagen = math_stroke_kagen_accle(cmd_velocity);
		if(ret < stroke_kagen) ret = stroke_kagen;

		str_stroke_process.data = "accel";
		pub_stroke_process_.publish(str_stroke_process);

		send_step_ = accle_stroke_step;
		pid_params_.set_stroke_prev(ret);

		return ret;
	}

	double _brake_stroke_pid_control(double current_velocity, double cmd_velocity, double acceleration)
	{
		double brake_stroke_step = setting_.brake_stroke_step_max;//2;
		double vel_sa = current_velocity - cmd_velocity;
		double brake_stroke_adjust_th = (current_velocity + cmd_velocity)/2.0 * (setting_.brake_stroke_adjust_th / 100.0);

		if(vel_sa < brake_stroke_adjust_th)
		//if(vel_sa < setting_.brake_stroke_adjust_th)
		{
			//brake_stroke_step -= (setting_.brake_stroke_adjust_th-vel_sa)*(setting_.accel_stroke_step_max-1)/setting_.brake_stroke_adjust_th;
			brake_stroke_step -= (setting_.brake_stroke_adjust_th-vel_sa)*(setting_.brake_stroke_step_max-1)/brake_stroke_adjust_th;
			if(brake_stroke_step < 1) brake_stroke_step = setting_.brake_stroke_step_min;
		}
		bool use_step_flag = true;

		const double velocity_magn = 1.7;
		double stopper_distance_th = (setting_.stopper_distance1 > cmd_velocity*velocity_magn) ? setting_.stopper_distance1 : cmd_velocity*velocity_magn;
		if(pid_params_.get_stroke_prev() > 0 && (stopper_distance_.distance < 0 || stopper_distance_.distance > stopper_distance_th))
		{
			pid_params_.clear_diff_velocity();
			pid_params_.clear_diff_acceleration();
			pid_params_.clear_diff_distance();
			pid_params_.set_accel_e_prev_velocity(0);
			pid_params_.set_accel_e_prev_acceleration(0);
			if(current_velocity > cmd_velocity)
			{
				double stroke_kagen = math_stroke_kagen_brake(cmd_velocity);
				std::cout << "kagen : " << stroke_kagen << std::endl;
				double stroke = pid_params_.get_stroke_prev()-brake_stroke_step;
				if(stroke < stroke_kagen) stroke = stroke_kagen;
				pid_params_.set_stroke_prev(stroke);
				std_msgs::String process;
				process.data = "brake 1";
				pub_stroke_process_.publish(process);
				return stroke;
			}
			//else return pid_params_.get_stroke_prev();
		}

		//アクセルからブレーキに変わった場合、Iの積算値をリセット
		//setting_.pedal_center_voltageをペダル中央電圧値として、現在の電圧との差が０位上の場合はアクセル側と判断する
		double stroke = setting_.pedal_center_voltage - can_receive_503_.pedal_voltage;
		//if (stroke > 0)
		{
			std::cout << "ACCEL_PEDAL_STROKE_OFFSET_" << std::endl;
			//pid_params.set_brake_e_prev_velocity(0);
			//pid_params.set_brake_e_prev_acceleration(0);
			pid_params_.set_accel_e_prev_velocity(0);
			pid_params_.set_accel_e_prev_acceleration(0);
			pid_params_.set_stroke_prev(0);
			//return 0;
		}


		//velocity PID
		//P
		double e = -1 * (cmd_velocity - current_velocity);
		std::cout << "if : " << cmd_velocity << "," << current_velocity << "," << e << std::endl;
		// since this is braking, multiply -1.
		if (e > 0 && e <= 1) { // added @ 2016/Aug/29
			e = 0;
			pid_params_.clear_diff_velocity();
		}
		std::cout << "e " << e << std::endl;
		//I
		double e_i, plus_e;
		if(stopper_distance_.fixed_velocity >= 0) plus_e = e;
		else
		{
			switch(stopper_distance_.send_process)
			{
				case autoware_msgs::StopperDistance::TEMPORARY_STOPPER:
					if(-acceleration2_twist_ > config_temporary_stopper_.deceleration) plus_e = -e;
					else plus_e = e;
					break;
				case autoware_msgs::StopperDistance::OBSTACLE:
					if(-acceleration2_twist_ > config_velocity_set_.deceleration_obstacle) plus_e = -e;
					else plus_e = e;
					break;
				case autoware_msgs::StopperDistance::SIGNAL:
					if(-acceleration2_twist_ > config_lane_rule_.acceleration) plus_e = -e;
					else plus_e = e;
					break;
				default:
					plus_e = e;
			}
		}
		pid_params_.plus_brake_diff_sum_velocity(plus_e);
		double brake_max_i = setting_.brake_max_i;
		if (pid_params_.get_brake_diff_sum_velocity() > brake_max_i)
			e_i = brake_max_i;
		else
			e_i = pid_params_.get_brake_diff_sum_velocity();

		//D
		double e_d = e - pid_params_.get_brake_e_prev_velocity();

		double target_brake_stroke = setting_.k_brake_p_velocity * e +
				setting_.k_brake_i_velocity * e_i +
				setting_.k_brake_d_velocity * e_d;
		pid_params_.set_brake_e_prev_velocity(e);

		stopper_distance_flag_ = 0;
		int a = (use_stopper_distance_) ? 1 : 0;
		std::cout << "use_stopper_distance " << a << ",  fixed_velocity " << stopper_distance_.fixed_velocity << std::endl;
		use_stopper_distance_val_for_brake_ = (use_stopper_distance_ == true) ? 1 : 0;
		stopper_distance_fixed_velocity_for_brake_ = stopper_distance_.fixed_velocity;
		stopper_distance_send_process_for_brake_ = stopper_distance_.send_process;

		std::string str_stopper_distance = "";
		std::cout << "stop_info," << stopper_distance_.fixed_velocity << "," << +stopper_distance_.send_process << std::endl;
		if(use_stopper_distance_ == true && (stopper_distance_.fixed_velocity <= 0 || stopper_distance_.send_process == 3))
		{stopper_distance_flag_ = 1;
			if(stopper_distance_.distance >= setting_.stopper_distance2 && stopper_distance_.distance <= setting_.stopper_distance1)
			{
				std::cout << loop_counter_ << " : stopD1" << std::endl;
				stop_distance_over_sum_ = 0;
				/*std::cout << "tbs," << target_brake_stroke;
				double d = stop_stroke - target_brake_stroke;
				if(d < 0) d = 0;
				target_brake_stroke  += d * (1 - stopper_distance_/ 20.0 );
				std::cout << ",tbs," << target_brake_stroke << ",d," << d << ",dis," << stopper_distance_ << std::endl;*/
				str_stopper_distance = "stopper1";
			}
			else if(stopper_distance_.distance >= setting_.stopper_distance3 && stopper_distance_.distance <= setting_.stopper_distance2)
			{
				std::cout << loop_counter_ << "stopD2" << std::endl;
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
					target_brake_stroke = pid_params_.get_stop_stroke_prev();
				}*/
				//if(temporary_fixed_velocity_ > 0)
				
				/*if(stopper_distance_.fixed_velocity > 0)//速度ありの停止線用処理
				{
					target_brake_stroke = pid_params_.get_stop_stroke_prev();
					if(current_velocity <= 0.5)
					{
						target_brake_stroke -= brake_stroke_step;
						if(target_brake_stroke < 0) target_brake_stroke = 0;
					}
				}*/
				str_stopper_distance = "stopper2";
			}
			else if(stopper_distance_.distance >= 0 && stopper_distance_.distance <= setting_.stopper_distance3)
			{std::cout << loop_counter_ << "stopD3" << std::endl;
				//if(temporary_fixed_velocity_ == 0)
				if(stopper_distance_.fixed_velocity == 0)
				{
					//target_brake_stroke = 0.0 + 500.0 * pow((2.0-distance)/2.0,0.5);
					brake_stroke_step = 0.5;
					target_brake_stroke = (200) * (setting_.stopper_distance3 - stopper_distance_.distance) / setting_.stopper_distance3;
					std::cout << "temporary_target_stroke," << target_brake_stroke << std::endl;
					//if(target_brake_stroke < pid_params_.get_stop_stroke_prev())
					//	target_brake_stroke = pid_params_.get_stop_stroke_prev();
					//if(target_brake_stroke == -(200) && can_receive_502_.velocity_mps != 0) stop_distance_over_sum_ += stop_distance_over_add_;
					//if(target_brake_stroke == -setting_.stop_stroke_max && current_velocity > 0.025) stop_distance_over_sum_ += stop_distance_over_add_;
					//target_brake_stroke += stop_distance_over_sum_;
					//std::cout << "temporary_target_stroke2," << target_brake_stroke << std::endl;
				}
				else stop_distance_over_sum_ = 0;
				str_stopper_distance = "stopper3";
			}
		}
		else stop_distance_over_sum_ = 0;

		double pedal_stroke_center = setting_.pedal_stroke_center;

		double ret = target_brake_stroke;//std::cout << "ret " << setting_.k_brake_p_velocity << std::endl;
		if (-ret < setting_.pedal_stroke_min)
			ret = -setting_.pedal_stroke_min;
		else if (-ret > pedal_stroke_center)
			ret = -pedal_stroke_center;
		std::cout << "brake_ret1," << ret << "," << setting_.pedal_stroke_min << "," << pedal_stroke_center << std::endl;

		if(use_step_flag == true)
		{
			/*if(pid_params_.get_stop_stroke_prev() < 0 && ret >= 0)
			{
				double tmp = pid_params_.get_stop_stroke_prev() + brake_stroke_step;
				if(tmp < ret) ret = tmp;
			}*/
			
			if(pid_params_.get_stop_stroke_prev()-ret >= 50 && pid_params_.get_stop_stroke_prev() >= 0)
			{
				ret = pid_params_.get_stop_stroke_prev();
				ret -= brake_stroke_step;
				if(ret < 0) ret = 0;
				//std::cout << "brake_ret 1" << std::endl;
			}
			//ブレーキをゆっくり踏む
			else if(pid_params_.get_stop_stroke_prev() > 0.0 && pid_params_.get_stop_stroke_prev() < ret)
			{
				double tmp = pid_params_.get_stop_stroke_prev() - brake_stroke_step;
				if(tmp > ret) ret = tmp;
				//if(-ret < setting_.pedal_stroke_min) ret = -setting_.pedal_stroke_min;
				//std::cout << "brake_ret 2" << std::endl;
			}
			else std::cout << "brake_ret 3" << std::endl;
		}

		//if(ret < 0) ret = 0;
		//if(ret > -setting_.pedal_stroke_min) ret = -setting_.pedal_stroke_min;
		short cap = (brake_stroke_cap_ > setting_.pedal_stroke_min) ? brake_stroke_cap_ : setting_.pedal_stroke_min;//std::max(brake_stroke_cap_, setting_.pedal_stroke_min);
		if(-ret < cap) ret = -cap;
		send_step_ = brake_stroke_step;
		pid_params_.set_stop_stroke_prev(ret);
		std_msgs::String str_process;
		str_process.data = "brake " + str_stopper_distance;
		pub_stroke_process_.publish(str_process);
		std::cout << "brake_ret2," << ret << std::endl;
		return -ret;
	}

	short _stopping_control(double current_velocity)
	{std::cout << "stop cur : " << current_velocity << std::endl;
		if (current_velocity < (VELOCITY_ZERO_VALUE_+20)/100.0)
		{
			int gain = (int)(((double)setting_.pedal_stroke_min)*can_receive_502_.cycle_time);
			std::cout << "stop  gain : " << gain << std::endl;
			std::cout << "cycle time : " << can_receive_502_.cycle_time << std::endl;
			std::cout << "stroke min : " << setting_.pedal_stroke_min << std::endl;
			double ret = pid_params_.get_stop_stroke_prev() + gain;
			if((int)ret > setting_.pedal_stroke_min) ret = setting_.pedal_stroke_min;
			if((int)ret < setting_.brake_stroke_stopping_med) ret = setting_.brake_stroke_stopping_med;
			return ret;
		}
		else
		{
			return setting_.brake_stroke_stopping_med;
		}
	}

	double _keep_control()
	{
		if(routine_.data == "accel")
		{
			pid_params_.set_stop_stroke_prev(0);
			return pid_params_.get_stroke_prev();
		}
		else if(routine_.data == "brake")
		{
			pid_params_.set_stroke_prev(0);
			return -pid_params_.get_stop_stroke_prev();
		}
		else if(routine_.data == "keep")
		{
			if(pid_params_.get_stroke_prev() > 0)
			{
				double accel = pid_params_.get_stroke_prev();
				accel -= 2;
				if(accel < 0) accel = 0;
				pid_params_.set_stroke_prev(accel);
				return accel;
			}
			else
			{
				double brake = pid_params_.get_stop_stroke_prev();
				brake -= 2;
				if(brake < 0) brake = 0;
				pid_params_.set_stop_stroke_prev(brake);
				return -brake;
			}
		}
		return 0;
	}

	void bufset_drive(unsigned char *buf, double current_velocity, double acceleration, double stroke_speed)
	{
		ros::Time nowtime = ros::Time::now();

		double cmd_velocity = vehicle_cmd_.ctrl_cmd.linear_velocity * 3.6;
		if(input_drive_mode_ == true && can_receive_501_.drive_auto)
			cmd_velocity = input_drive_ / 100.0;
		double accel_mode_avoidance_distance = 30;//(current_velocity > accel_avoidance_distance_min_) ? current_velocity : accel_avoidance_distance_min_;

		std::cout << "twist.ctrl_cmd " << vehicle_cmd_.ctrl_cmd.linear_velocity * 3.6 << ", cmd_velocity " << cmd_velocity << std::endl;

		std::cout << "auto_mode" << std::endl;
		std::cout << "cur_cmd : " << current_velocity << "," << cmd_velocity << "," << setting_.velocity_limit << std::endl;

		std::cout << "accel_avoidance_distance_min : " << accel_avoidance_distance_min_ << std::endl;
		std::cout << "velocity hikaku : " << cmd_velocity << "," << current_velocity << std::endl;
		std::cout << "flag : " << (int)checkMobileyeObstacleStop(nowtime) << "," << stopper_distance_.distance << "," << in_accel_mode_ << std::endl;

		//加速判定
		if (checkMobileyeObstacleStop(nowtime) == false
					&& fabs(cmd_velocity) > current_velocity + setting_.acceptable_velocity_variation
			        && current_velocity < setting_.velocity_limit
			        && (stopper_distance_.distance<0 || stopper_distance_.distance>accel_mode_avoidance_distance)
					&& in_accel_mode_ == true)
		{
			std::cout << " stroke drive" << std::endl;
			pid_params_.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_ACCEL_);
		}
		//減速判定
		else if(fabs(cmd_velocity) < current_velocity - setting_.acceptable_velocity_variation
					&& fabs(cmd_velocity) > 0.0 || (stopper_distance_.distance>=0 && stopper_distance_.distance <=current_velocity)
					&& in_brake_mode_ == true)
		{
			std::cout << "stroke brake" << std::endl;
			pid_params_.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_BRAKE_);
		}
		//停止線判定
		else if (stopper_distance_.distance >= 0 && stopper_distance_.distance < accel_avoidance_distance_min_ && in_brake_mode_ == true)
		{
			std::cout << "stroke distance" << std::endl;
			pid_params_.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_BRAKE_);
		}
		else if(current_velocity > setting_.velocity_limit)
		{
			std::cout << "stroke over limit" << std::endl;
			pid_params_.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_KEEP_);
		}
		else {
			std::cout << "keep" << std::endl;
			pid_params_.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_KEEP_);
		}

		double new_stroke = 0;
		switch(pid_params_.get_stroke_state_mode_())
		{
		case PID_params::STROKE_STATE_MODE_ACCEL_:
			new_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity);//, &stroke_speed);
			routine_.data = "accel";
			pub_stroke_routine_.publish(routine_);
			break;
		case PID_params::STROKE_STATE_MODE_BRAKE_:
			new_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity, acceleration);//, &stroke_speed);
			/*if(stopper_distance_ >= 0 && stopper_distance_ <= 1.5 &&
				new_stroke > pid_params_.get_stroke_prev())
					new_stroke = pid_params_.get_stroke_prev();*/
			routine_.data = "brake";
			pub_stroke_routine_.publish(routine_);
			break;
		case PID_params::STROKE_STATE_MODE_STOP_:
			new_stroke = _stopping_control(current_velocity);
			pid_params_.set_stop_stroke_prev(new_stroke);
			break;
		case PID_params::STROKE_STATE_MODE_KEEP_:
			new_stroke = _keep_control();//pid_params_.get_stroke_prev();
			routine_.data = "keep";
			pub_stroke_routine_.publish(routine_);
			break;
		}

		//AUTOモードじゃない場合、stroke値0をcanに送る
		if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO ||
			can_receive_503_.clutch == false)
		{
			pid_params_.clear_diff_velocity();
			pid_params_.clear_diff_acceleration();
			pid_params_.clear_diff_distance();
			pid_params_.set_stop_stroke_prev(0);
			pid_params_.set_stroke_prev(0);
			//short drive_val = 0;
			//unsigned char *drive_point = (unsigned char*)&drive_val;
			//buf[4] = drive_point[1];  buf[5] = drive_point[0];
			new_stroke = 0;
			std::cout << "manual_mode" << std::endl;
			return;
		}

		short input_stroke = (short)new_stroke;
		if(input_drive_mode_ == true) input_stroke = input_drive_;
		unsigned char *drive_point = (unsigned char*)&input_stroke;
		buf[4] = drive_point[1];  buf[5] = drive_point[0];

		pseudo_params_.stroke_actual = input_stroke;

		//brakeキャップ値の上昇
		brake_stroke_cap_ -= 2;
		brake_stroke_cap_ = (brake_stroke_cap_ > setting_.pedal_stroke_min) ? brake_stroke_cap_ : setting_.pedal_stroke_min;
		//brake_stroke_cap_  std::max(brake_stroke_cap_, (double)setting_.pedal_stroke_min);
	}

	void bufset_car_control(unsigned char *buf, double current_velocity)
	{
		buf[6] = buf[7] = 0;

		if(emergency_stop_ == 0x2) {buf[6] |= 0x80;  emergency_stop_ = 0;}
		else if(emergency_stop_ == 0x1) {buf[6] |= 0x40;  emergency_stop_ = 0;}
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
			if(time > blinker_right_time_)  blinker_right_ = false;
		}
		else if(blinker_left_ == true)
		{
			buf[6] |= 0x01;
			ros::Time time = ros::Time::now();
			if(time > blinker_left_time_)  blinker_left_ = false;
		}
		else if(blinker_stop_ == true)
		{
			buf[6] |= 0x03;
			ros::Time time = ros::Time::now();
			if(time > blinker_stop_time_)  blinker_stop_ = false;
		}
		if(light_high_ == true) buf[7] |= 0x20;

		if (shift_auto_ == true)
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
		}
	}
public:
	MicrobusPseudoCanSender(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
		, flag_drive_mode_(false)
		, flag_steer_mode_(false)
		, input_drive_mode_(true)
		, input_steer_mode_(true)
		, input_steer_(0)
		, input_drive_(0)
		, angle_limit_over_(false)
		, accel_avoidance_distance_min_(30)
		, in_accel_mode_(true)
		, in_brake_mode_(true)
		, stop_distance_over_add_(10.0/100.0)
		, stop_distance_over_sum_(0)
		, acceleration1_twist_(0)
		, acceleration2_twist_(0)
		, jurk1_twist_(0)
		, jurk2_twist_(0)
		, stopper_distance_flag_(0)
		, use_stopper_distance_(true)
		, loop_counter_(0)
		, emergency_stop_(false)
		, drive_clutch_(true)
		, steer_clutch_(true)
		, interface_lock_(false)
		, blinker_right_(false)
		, blinker_left_(false)
		, blinker_stop_(false)
		, light_high_(false)
		, shift_auto_(false)
		, shift_position_(0)
		, steer_override_value_(STEER_OVERWRIDE_TH)
	{
		pub_microbus_can_sender_status_ = nh_.advertise<autoware_can_msgs::MicroBusCanSenderStatus>("/microbus/can_sender_status", 1, true);
		pub_stroke_process_ = nh_.advertise<std_msgs::String>("/microbus/stroke_process", 1);
		pub_stroke_routine_ = nh_.advertise<std_msgs::String>("/microbus/stroke_routine", 1);
		pub_vehicle_status_ = nh_.advertise<autoware_msgs::VehicleStatus>("/microbus/vehicle_status", 1);
		pub_pseudo_params_ = nh_.advertise<autoware_can_msgs::MicroBusPseudoParams>("/microbus/pseudo_params", 1);
		pub_tmp_ = nh_.advertise<std_msgs::String>("/microbus/tmp", 1);

		sub_config_ = nh_.subscribe("/config/microbus_can", 10, &MicrobusPseudoCanSender::callbackConfigMicroBusCan, this);
		sub_config_temporary_stopper_ = nh_.subscribe("/config/temporary_stopper", 10, &MicrobusPseudoCanSender::callbackConfigTemporaryStopper, this);
		sub_config_velocity_set_ = nh_.subscribe("/config/velocity_set_modification", 10, &MicrobusPseudoCanSender::callbackConfigVelocitySet, this);
		sub_config_lane_rule_ = nh_.subscribe("/config/lane_rule", 10, &MicrobusPseudoCanSender::callbackConfigLaneRule, this);
		sub_microbus_can_501_ = nh_.subscribe("/microbus/can_receive501", 10, &MicrobusPseudoCanSender::callbackMicrobusCan501, this);
		sub_microbus_can_502_ = nh_.subscribe("/microbus/can_receive502", 10, &MicrobusPseudoCanSender::callbackMicrobusCan502, this);
		sub_microbus_can_503_ = nh_.subscribe("/microbus/can_receive503", 10, &MicrobusPseudoCanSender::callbackMicrobusCan503, this);
		sub_vehicle_cmd_ = nh_.subscribe("/vehicle_cmd", 10, &MicrobusPseudoCanSender::callbackVehicleCmd, this);
		sub_microbus_drive_mode_ = nh_.subscribe("/microbus/drive_mode_send", 10, &MicrobusPseudoCanSender::callbackDModeSend, this);
		sub_microbus_steer_mode_ = nh_.subscribe("/microbus/steer_mode_send", 10, &MicrobusPseudoCanSender::callbackSModeSend, this);
		sub_stopper_distance_ = nh_.subscribe("/stopper_distance", 10, &MicrobusPseudoCanSender::callbackStopperDistance, this);
		sub_emergency_stop_ = nh_.subscribe("/microbus/emergency_stop", 10, &MicrobusPseudoCanSender::callbackEmergencyStop, this);
		sub_drive_clutch_ = nh_.subscribe("/microbus/drive_clutch", 10, &MicrobusPseudoCanSender::callbackDriveClutch, this);
		sub_steer_clutch_ = nh_.subscribe("/microbus/steer_clutch", 10, &MicrobusPseudoCanSender::callbackSteerClutch, this);
		sub_interface_lock_ = nh_.subscribe("/microbus/interface_lock", 10, &MicrobusPseudoCanSender::callbackInterfaceLock, this);
		sub_automatic_door_ = nh_.subscribe("/microbus/automatic_door", 10, &MicrobusPseudoCanSender::callbackAutomaticDoor, this);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 10, &MicrobusPseudoCanSender::callbackWaypointParam, this);
		sub_steer_override_ = nh.subscribe("/microbus/steer_override", 10 , &MicrobusPseudoCanSender::callbackSteerOverride, this);

		sub_current_pose_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/current_pose", 10);
		sub_current_velocity_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, "/current_velocity", 10);
		sync_twist_pose_ = new message_filters::Synchronizer<TwistPoseSync>(TwistPoseSync(SYNC_FRAMES), *sub_current_velocity_, *sub_current_pose_);
		sync_twist_pose_->registerCallback(boost::bind(&MicrobusPseudoCanSender::TwistPoseCallback, this, _1, _2));

		stopper_distance_.distance = -1;
		stopper_distance_.send_process = autoware_msgs::StopperDistance::UNKNOWN;

		waypoint_param_.blinker = 0;
		automatic_door_time_ = blinker_right_time_ = blinker_left_time_ =
				blinker_stop_time_ = ros::Time::now();

		pid_params_.init(0.0);
	}

	~MicrobusPseudoCanSender()
	{
		delete sync_twist_pose_;
		delete sub_current_pose_;
		delete sub_current_velocity_;
	}

	void run()
	{
		ros::Time nowtime = ros::Time::now();
		double current_velocity = current_velocity_.twist.linear.x * 3.6;
		double acceleration = 0;

		unsigned char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};
		bufset_mode(buf);
		bufset_steer(buf);
		bufset_drive(buf, current_velocity, acceleration, 2.0);
		bufset_car_control(buf, current_velocity);

		autoware_msgs::VehicleStatus status;
		status.header.stamp = nowtime;
		status.drivemode = (buf[0] & 0x0B != 0x0B) ? autoware_msgs::VehicleStatus::MODE_AUTO : autoware_msgs::VehicleStatus::MODE_MANUAL;
		status.steeringmode = (buf[0] & 0xA0 != 0xA0) ? autoware_msgs::VehicleStatus::MODE_AUTO : autoware_msgs::VehicleStatus::MODE_MANUAL;
		status.current_gear.gear = autoware_msgs::Gear::NONE;
		status.lamp = 0;
		status.light = 0;
		status.speed = current_velocity_.twist.linear.x;
		//if(can_receive_502_.angle_actual > 0) status.angle = can_receive_502_.angle_actual / wheelrad_to_steering_can_value_left;
		//else status.angle = can_receive_502_.angle_actual / wheelrad_to_steering_can_value_right;
		int16_t angle_val = (waypoint_param_.mpc_target_input == 0) ? can_receive_502_.angle_actual : target_steer_;
		if(can_receive_502_.angle_actual > 0) status.angle = angle_val / wheelrad_to_steering_can_value_left;
		else status.angle = angle_val / wheelrad_to_steering_can_value_right;
		pub_vehicle_status_.publish(status);

		autoware_can_msgs::MicroBusCanVelocityParam vparam;
		vparam.header.stamp = nowtime;
		vparam.velocity = current_velocity_.twist.linear.x;
		vparam.acceleration = acceleration2_twist_;
		vparam.jurk = jurk2_twist_;
		vparam.steer_angle_deg = can_receive_502_.angle_deg;
		vparam.steer_angle_acc_deg = can_receive_502_.angle_deg_acc;
		pub_velocity_param_.publish(vparam);

		pseudo_params_.header.stamp = nowtime;
		pub_pseudo_params_.publish(pseudo_params_);

		std::stringstream str_pub;
		str_pub << +waypoint_param_.mpc_target_input << "," << angle_val;
		pub_tmp_.publish(str_pub.str());

		loop_counter_++;
	}
};


int main(int argc, char** argv)
{
   	ros::init(argc, argv, "microbus_pseudo_can_sender");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	MicrobusPseudoCanSender sender(nh, private_nh);
	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		sender.run();
		loop_rate.sleep();
	}
	return 0;
}