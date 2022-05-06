//mobileyeのobstacle_statusは
//  0:undefined
//  1:停車中(ブレーキランプがついている)
//  2:停止中(動き出す可能性あり)
//  3:移動中
//  4:対向車
//  5:駐車中(ブレーキランプがついていない)
//  6:使用してない

#include <ros/ros.h>
#include <limits.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <autoware_config_msgs/ConfigMobileyeTracker.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/TransformMobileyeObstacle.h>
#include <autoware_msgs/MobileyeCmdParam.h>
#include <autoware_msgs/CarCruiseStatus.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include "mobileye_obstacle_info.h"

//waypoint２点と車両位置のy方向距離を計算
double y_distanbce(const geometry_msgs::Point pose1, const geometry_msgs::Point pose2, const geometry_msgs::Point cur)
{
	double x1 = pose1.x, x2 = pose2.x;
	double y1 = pose1.y, y2 = pose2.y;
	double a = y2 - y1;
	double b = x1 - x2;
	double c = - x1 * y2 + y1 * x2;

	//double x0 = current_pose_.pose.position.x, y0 = current_pose_.pose.position.y;
	double x0 = cur.x, y0 = cur.y;
	double db = sqrt(a * a + b * b);
	if(db == 0)
	{
		std::cout << "pose1とpose2が同じ" << std::endl;
		return 998;
	}
	return (a * x0 + b * y0 + c) / db;
}

double euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
	double x1 = p1.x,  x2 = p2.x;
	double y1 = p1.y,  y2 = p2.y;
	double z1 = p1.z,  z2 = p2.z;
	double xd = x1 - x2,  yd = y1 - y2,  zd = z1 - z2;
	return std::sqrt(xd*xd + yd*yd + zd*zd);
}

/*double mathAcc(const std::vector<autoware_msgs::TransformMobileyeObstacle> &vel_list)
{
	if(vel_list.size() <= 2) return 0;
	double sum = 0;
	for(size_t i=1; i<vel_list.size()-1; i++)
	{
		const double twist1 = vel_list[i-1].velocity_mps;
		const double twist2 = vel_list[i+1].velocity_mps;
		double vel_diff = twist2 - twist1;
		ros::Duration ros_time_diff = vel_list[i+1].header.stamp - vel_list[i-1].header.stamp;
		double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
		sum += vel_diff / time_diff;
	}
	return sum / (vel_list.size()-2);
}*/

//自車と先行車両の車間距離の計算
//mps : 指定車両の速度
//target_deceleration : 目標減速度
//delay_time : 前方車両を感知してからブレーキがかかるまでの距離
//min_free_running_distance : 空走距離
double targetFreeSpace(const double mps, const double target_deceleration, const double delay_time, const double min_free_running_distance)
{
	//double mps = kmh / 3.6;
	double free_running_distance = mps * delay_time;
	double brakeing_distace = mps * mps / (2 * target_deceleration * 9.8);
	return std::max(min_free_running_distance, free_running_distance) + brakeing_distace;
}

//自車と先行車両の車間距離の計算
//cur_mps : 自車両の速度
//front_mps : 前方車両の速度
//cur_target_deceleration : 自車両の目標減速度 数値が大きくなると急ブレーキを意味する
//front_target_deceleration : 前方車両の目標減速度 数値が大きくなると急ブレーキを意味する
//cur_delay_time : 自車両の前方車両を感知してからブレーキがかかるまでの距離
//front_delay_time : 前方車両の前方車両を感知してからブレーキがかかるまでの距離
//min_free_running_distance : 空走距離
double targetFreeSpaceRSS(const double cur_mps, const double front_mps, const double cur_target_deceleration, const double front_target_deceleration,
	const double cur_delay_time, const double front_delay_time, const double min_free_running_distance, double &cur_space_ret, double &front_spalce_ret)
{
	double cur_free_running_distance = cur_mps * cur_delay_time;
	double front_free_running_distance = front_mps * front_delay_time;
	double cur_brakeing_distace = cur_mps * cur_mps / (2 * cur_target_deceleration * 9.8);
	double front_brakeing_distace = front_mps * front_mps / (2 * front_target_deceleration * 9.8);
	double cur_space = cur_free_running_distance + cur_brakeing_distace;
	double front_space = front_free_running_distance + front_brakeing_distace;
	double free_running_distance_corr = 0;//cur_mps*3.6/2;//空走距離の速度補正
	cur_space_ret = cur_space;
	front_spalce_ret = front_space;
	double rss = cur_space - front_space + free_running_distance_corr + min_free_running_distance;
	double Lt = targetFreeSpace(cur_mps, cur_target_deceleration, cur_delay_time, min_free_running_distance);
	return std::max(std::min(Lt, rss), min_free_running_distance);
}

/*class CmdAcc
{
public:
	bool exist;
	double cmd_acc_;//!<指令加速度
	double target_scheduled_achievement_time_;//!<達成予定時刻
	double Lt_;//!<目標車間距離
	double L0_;//!<現在車間距離
	double current_velocity_;//!<自車両の速度
	double vehicle_ahead_velocity_;//!<前方車両の速度
	double current_acc_;//!<自車両の速度
	double vehicle_ahead_acc_;//!<前方車両の加速度(mobileye param)
	double vehicle_ahead_acc2_;//!<前方車両の加速度(差分)
	double relative_vel_;//!<自車両と前方車両との相対速度
	double relative_acc_;//!<自車両と前方車両との相対加速度
	void print() const
	{
		std::cout << "指令加速度," << cmd_acc_ << "m/s^2" << std::endl;
		std::cout << "達成予定時刻," << target_scheduled_achievement_time_ << "秒"<< std::endl;
		std::cout << "目標車間距離," << Lt_ << "m" << std::endl;
		std::cout << "現在車間距離," << L0_ << "m" << std::endl;
		std::cout << "自車両の速度," << current_velocity_<< "m/s" << std::endl;
		std::cout << "前方車両の速度," << vehicle_ahead_velocity_<< "m/s" << std::endl;
		std::cout << "相対速度," << relative_vel_<< "m/s" << std::endl;
		std::cout << "自車両の加速度," << current_acc_ << "m/s^2" << std::endl;
		std::cout << "前方車両の加速度," << vehicle_ahead_acc_ << "m/s^2" << std::endl;
		std::cout << "前方車両の加速度2," << vehicle_ahead_acc2_ << "m/s^2" << std::endl;
		std::cout << "相対加速度," << relative_acc_ << "m/s^2" << std::endl;
		std::cout << "t秒後の自車両速度," << current_velocity_ + cmd_acc_ * target_scheduled_achievement_time_ << "m/s" << std::endl;
		std::cout << "t秒後の前方車両速度," << vehicle_ahead_velocity_ + (cmd_acc_ + relative_acc_) * target_scheduled_achievement_time_ << "m/s" << std::endl;
		std::cout << "t秒後の車間距離," << L0_ + relative_vel_ * target_scheduled_achievement_time_ + 1.0/2.0 * relative_acc_ * std::pow(target_scheduled_achievement_time_, 2) << "m" << std::endl;
	}
};*/

//停止線を設定するwaypoint情報
struct StopLineInfo
{
	int waypoint_index_; //!< 停止線を設定する経路ID
	int x_adj_; //!< 進行方向に関する微調整
	double distance_; //!<最初のwaypointから前方車両までの距離
};

class CarTrackinig
{
private:
	//const static double JUDG_OBJ_POSEX = 100; //!< 正面判定の車両相対X座標(進行方向)
	//const static double JUDG_OBJ_POSEY = 1.5; //!< 正面判定の車両相対Y座標(横方向)
	//const static double OBS_DELETE_TIME = 1.0; //!<オブジェクトの更新がなかった場合にデーターから消去する秒数

	const static size_t CURRENT_VELOCITY_LIST_SIZE = 5; //!< current_velocityの最大サイズ
	const static double MAX_ACCEPTABLE_REL_VEL = -20.0; //!< 許容される相対速度の最大値
	const static double MAX_DECELERATION = 0.9; //!<最大減速度
	const static int16_t MIN_CAN_ACCEL_STROKE_CAP = 250;//!<CANに通知するアクセルSTROKEキャップの最小値
	const static int16_t MAX_CAN_ACCEL_STROKE_CAP = 500;//!<CANに通知するアクセルSTROKEキャップの最大値
	const static double CAN_SEND_TH_KMH = 10.0;//8.0;//!<アクセルSTROKEキャップ通知の前方車両速度に対するしきい値(時速)
	const static double CAN_SEND_TH_ACC = 0.3;//0.3;//!<アクセルSTROKEキャップ通知の相対加速度に対するしきい値(m/s2)
	const static double CAN_SEND_TH_DISTANCE = 40.0;//!<アクセルSTROKEキャップ通知の前方車両距離に対するしきい値(m)
	const static double LT_RSS_TH_MAGN = 1.375;//責任車間距離をしきい値として運用する場合の倍率

	//const static double min_free_running_distance_ = 2.0; //!< 最小車間距離
	//const static double target_deceleration_ = 0.3; //!<目標減速度
	//const static double control_cycle_ = 10.0; //!<制御周期
	//const static double delay_time_ = 1.0 / control_cycle_ * 2.0; //!< 1/制御周期 * 2

	const static size_t PREV_TRAKING_SIZE = 10;
	/*const static int TRACKING_NO = 0;
	const static int TRACKING_DECELERATION1 = 1;
	const static int TRACKING_DECELERATION2 = 2;
	const static int TRACKING_EMG_DECELERATION = 3;
	const static int TRACKING_STOP = 4;
	const static int TRACKING_ACCELERATION = 5;*/

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber sub_config_;
	ros::Subscriber sub_waypoints_;
	ros::Subscriber sub_current_pose_;
	ros::Subscriber sub_current_velocity_;
	ros::Subscriber sub_mobileye_obstacle_;//1<mobileyeからの車両情報(1車両ごと)
	ros::Subscriber sub_target_deceleration_;//!<目標減速度

	ros::Publisher pub_waypoints_;
	ros::Publisher pub_mobileye_cmd_param_;//!<速度計算確認用
	ros::Publisher pub_waypoint_vel_;
	ros::Publisher pub_track_pattern_;//!<前方車両に対する動作フラグをpublish
	ros::Publisher pub_front_mobileye_obstacle_;//!<前方車両と判断されたmobileye情報
	ros::Publisher pub_line_th_status_;
	ros::Publisher pub_can_stroke_cap_;//!<CANに通知するアクセルSTROKEキャップ
	ros::Publisher pub_car_cruise_status_;//!<前方車両追従の情報
	ros::Publisher pub_front_car_marker_;//前方車両のrviz表示用

	ros::Timer obs_delete_timer_;

	autoware_config_msgs::ConfigMobileyeTracker config_;
	std::vector<MobileyeObstacleInfo> mobileye_obstacle_info_list_; //!<mobileyeからの車両情報のリスト
	MobileyeObstacleInfo foreground_mobileye_; //!< 探索範囲内で見つかったmobileye正面オブジェクト
	//geometry_msgs::TwistStamped current_velocity_; //!< 現在の自車両速度
	std::vector<geometry_msgs::TwistStamped> current_velocity_list_; //!< current_velocityのリスト
	double current_velocity_ave_mps_; //!< 現在の自車両速度
	double current_acc_ave_mps_; //!<現在の加速度
	ros::Time velocity0_timer_; //!<障害物速度が最初に0と判断された時間
	double front_bumper_to_baselink_; //!< baselinkからフロントバンパーまでの距離
	double mobileye_virtual_bumper_; //!<mobileyeに設定してある仮想バンパーの距離 バスなどのボンネットがない車用
	double vehicle_width_;//!< 車両の幅
	geometry_msgs::PoseStamped current_pose_;
	double target_deceleration_;//!<目標減速度
	std::vector<int> prev_tracking_mode_;//!<過去のトラッキングモードのリスト
	double velocity_limit_of_stopmode_kmh_;//!<TRAKING_STOP初回時に決める経路速度限界

	double mathVelAve()
	{
		double sum = 0;
		for(const geometry_msgs::TwistStamped &twist : current_velocity_list_)
			sum += twist.twist.linear.x;
		return sum / (current_velocity_list_.size());
	}

	//autoware_msgs::MobileyeCmdParam mathTargetAcc(const autoware_msgs::TransformMobileyeObstacle obs) const
	autoware_msgs::MobileyeCmdParam mathTargetAcc(const MobileyeObstacleInfo &obs_list) const
	{
		const autoware_msgs::TransformMobileyeObstacle obs = obs_list.getLatestObs();//obs_list.getObs(obs_list.getSize()-1);

		double relative_vel = obs_list.getObsRelativeVelMpsAve();//obs.orig_data.obstacle_rel_vel_x;//前方車両と自車両との相対速度
		double relative_acc = obs_list.getObsRelativeAccAve();//obs.orig_data.object_accel_x;//前方車両と自車両との相対加速度
		double current_vel = current_velocity_ave_mps_;//current_velocity_.twist.linear.x;
		double current_acc = current_acc_ave_mps_;//mathAcc(current_velocity_list_);//自車両の加速度
		
		double vehicle_ahead_vel = obs_list.getVelocityMpsAve();//relative_vel + current_vel;//前方車両の速度
		double vehicle_ahead_acc = obs_list.getAccAve();//relative_acc + current_acc;//前方車両の加速度
		double vehicle_ahead_acc2 = 0;//mathAcc(foreground_mobileye_);
		//double Lt = targetFreeSpace(current_velocity_.twist.linear.x, target_deceleration_, delay_time_, min_free_running_distance_);
		double delay_time = 1.0;//1.0 / config_.control_cycle * 2.0;
		double Lt = targetFreeSpace(current_velocity_ave_mps_, target_deceleration_, delay_time, config_.min_free_running_distance);
//		double Lt_prime = targetFreeSpace(vehicle_ahead_vel, target_deceleration_, delay_time, config_.min_free_running_distance);
		double Lt_prime = targetFreeSpace(vehicle_ahead_vel, 0.3, delay_time*0.6, config_.min_free_running_distance);
		double L0_Lt = targetFreeSpace(vehicle_ahead_vel, MAX_DECELERATION, delay_time, config_.min_free_running_distance);
		double cur_space, front_space;
		//double Lt_RSS = targetFreeSpaceRSS(current_velocity_ave_mps_, vehicle_ahead_vel, 0.13, 0.3, delay_time, delay_time*0.6, 3.0, cur_space, front_space);
		double Lt_RSS = targetFreeSpaceRSS(current_velocity_ave_mps_, vehicle_ahead_vel, 0.14, 0.3, delay_time, delay_time*0.6, 3.0, cur_space, front_space);
		double L0 = obs_list.getRelativePoseXAve();//経路から計算した距離にすべき？//obs.orig_data.obstacle_pos_x;//前方車両までの距離
		double cmd_acc = vehicle_ahead_acc + (1.0 / 2.0) * std::pow(relative_vel, 2.0) / (Lt_RSS - L0);//(Lt - L0);
		double Lt_part1 = (vehicle_ahead_vel-current_vel)*(-(vehicle_ahead_vel-current_vel)/(vehicle_ahead_acc-cmd_acc));
		double Lt_part2 = 0.5*(vehicle_ahead_acc-cmd_acc)*std::pow(-(vehicle_ahead_vel-current_vel)/(vehicle_ahead_acc-cmd_acc),2.0);
		double Lt2 = L0 + Lt_part1 + Lt_part2;
		double target_scheduled_achievement_time = 2.0 * (Lt - L0) / (relative_vel);
		double expected_collision_time;// = (relative_vel == 0) ? 10000.0 : L0 / (-relative_vel);
		double reach_time;
		if(relative_vel >= 0)
		{
			expected_collision_time = DBL_MAX;//前方車両が近づかない場合は衝突予想時間を最大値にする
			reach_time = DBL_MAX;
		}
		else
		{
			//expected_collision_time = L0 / (-relative_vel);
			double sqrt_val = relative_vel * relative_vel + 2 * current_acc * L0;
			if(std::abs(current_acc) <= 1.0E-4 || sqrt_val < 0) expected_collision_time = L0 / (-relative_vel);
			else expected_collision_time = (relative_vel + std::sqrt(sqrt_val)) / current_acc;
		}
		//double cmd_acc = vehicle_ahead_acc + std::pow(relative_vel, 2.0) / (2*(Lt - L0));
		//double target_scheduled_achievement_time = - (relative_vel) / (vehicle_ahead_acc - cmd_acc);

		//CmdAcc cmdacc;
		autoware_msgs::MobileyeCmdParam cmdacc;
		cmdacc.exist = true;
		cmdacc.cmd_acc = cmd_acc;
		cmdacc.target_scheduled_achievement_time = target_scheduled_achievement_time;
		cmdacc.Lt = Lt;
		cmdacc.Lt_prime = Lt_prime;
		cmdacc.L0 = L0;
		cmdacc.L0_Lt = L0_Lt;
		cmdacc.Lt_RSS = Lt_RSS;
		cmdacc.obstacle_pos_x_diff = obs_list.getRelativePoseXDiffAve();
		cmdacc.cur_space = cur_space;
		cmdacc.front_space = front_space;
		cmdacc.current_velocity = current_velocity_ave_mps_;//current_vel;
		cmdacc.relative_vel = relative_vel;
		cmdacc.vehicle_ahead_velocity = vehicle_ahead_vel;//relative_vel + current_vel;
		cmdacc.current_acc = current_acc;
		cmdacc.vehicle_ahead_acc = vehicle_ahead_acc;
		cmdacc.vehicle_ahead_acc2 = vehicle_ahead_acc2;
		cmdacc.relative_acc = relative_acc;
		cmdacc.expected_collision_time = expected_collision_time;
		cmdacc.obs_velocity_kmh = obs.velocity_mps * 3.6;
		cmdacc.obstacle_status = obs.orig_data.obstacle_status;
		return cmdacc;
	}

	int trackJudge(const autoware_msgs::MobileyeCmdParam &cmd, const MobileyeObstacleInfo &obslist,//const autoware_msgs::TransformMobileyeObstacle &obs,
		//const double front_velocity_ave_mps,
		const double way_vel)
	{
		int ret = autoware_msgs::CarCruiseStatus::TRACKING_NO;

		/*if(config_.min_free_running_distance >= cmd.L0) //現在車間距離が最低車間距離以下なら停止
		{
			ret = TRACK_STOP;
		}
		else if(cmd.expected_collision_time <= 0) //前方車両衝突時間が0以下(衝突の危険なし)
		{
			ret = TRACK_NO;
		}
		else if(cmd.expected_collision_time > config_.expected_collision_time_th) //前方車両衝突時間がconfig_.expected_collision_time_th以上(衝突の危険ややあり)
		{
			if(cmd.L0 > cmd.Lt) ret = TRACK_NO;
			else ret = TRACK_GO;
		}
		else //前方車両衝突時間がconfig_.expected_collision_time_th以上(衝突の危険あり)
		{
			ret = TRACK_STOP;
		}
		return ret;*/

		//if(cmd.vehicle_ahead_velocity < 10.0 / 3.6) ret = TRACK_STOP;
		//std::cout << "TA," << cmd.relative_vel << "," << cmd.vehicle_ahead_velocity << " < " << way_vel << std::endl;

		const autoware_msgs::TransformMobileyeObstacle &obs = obslist.getLatestObs();
		//double foreg_velocity_mps = (cmd.L0 < 15) ? foreground_mobileye_.getLatestVelocityMps() : foreground_mobileye_.getVelocityMpsAve();//前方車両までの距離が一定以下の場合は、応答速度を考慮して平均を使用しない
		double foreg_velocity_mps = foreground_mobileye_.getVelocityMpsAve();
		double velocity_mps_th = (foreground_mobileye_.getAccAve() > 0.6) ? 2.0/3.6: 5.0/3.6;

		if(foreg_velocity_mps < velocity_mps_th || cmd.expected_collision_time < 10.0//cmd.relative_vel < MAX_ACCEPTABLE_REL_VEL / 3.6
			|| obs.orig_data.obstacle_status == mobileye_560_660_msgs::ObstacleData::OBSTACLE_STATUS_STOPPED
			|| obs.orig_data.obstacle_status == mobileye_560_660_msgs::ObstacleData::OBSTACLE_STATUS_PARKED
			|| obs.orig_data.obstacle_status == mobileye_560_660_msgs::ObstacleData::OBSTACLE_STATUS_STANDING)//停止
		{
			ros::Time nowtime = ros::Time::now();
			if(velocity0_timer_ == ros::Time(0)) velocity0_timer_ = nowtime;
			ros::Duration ros_time_diff = nowtime - velocity0_timer_;
			double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
			if(time_diff >= 0.1) ret = autoware_msgs::CarCruiseStatus::TRACKING_STOP;
		}
		else if(cmd.L0 > cmd.Lt_RSS * LT_RSS_TH_MAGN)//責任車間距離を基準とした一定距離よりも離れている場合は追跡処理をしない
		{
			ret = autoware_msgs::CarCruiseStatus::TRACKING_NO;
		}
		else
		{
			velocity0_timer_ = ros::Time(0);

			//if(cmd.Lt > cmd.L0 && cmd.Lt_prime < cmd.L0 && cmd.relative_vel < 0)
			//if(cmd.Lt > cmd.L0 && cmd.relative_vel < 0)
			if(cmd.Lt_RSS > cmd.L0 && cmd.relative_vel < 0)
			{
				ret = autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION1;//減速
			}
			//else if(cmd.Lt < cmd.L0 && cmd.relative_vel < 0)
			else if(cmd.Lt_RSS < cmd.L0 && cmd.relative_vel < 0)
			{
				ret = autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION2;//減速
			}
			//else if(cmd.L0 < cmd.Lt_prime && cmd.relative_vel < 0)
			//else if((cmd.L0 < cmd.L0_Lt || cmd.expected_collision_time < 3.0) && cmd.relative_vel < 0)
			else if(cmd.expected_collision_time < 5.0)
			{
				ret = autoware_msgs::CarCruiseStatus::TRACKING_EMG_DECELERATION;//緊急減速
			}
			else if(cmd.relative_vel > 0 && cmd.vehicle_ahead_velocity < way_vel)
			{
				ret = autoware_msgs::CarCruiseStatus::TRACKING_ACCELERATION;//前方車両の速度に合わせる
			}
		}

		return ret;
	}

	void prevTrakingAdd(int tracking_mode)
	{
		prev_tracking_mode_.push_back(tracking_mode);
		if(prev_tracking_mode_.size() > PREV_TRAKING_SIZE) prev_tracking_mode_.erase(prev_tracking_mode_.begin());
	}

	//前方車両速度から追跡経路の速度を作成
	void createCarTrackingWaypoints(const autoware_msgs::Lane::ConstPtr &lane, const StopLineInfo &stop_line_info)
	{
		std_msgs::String track_pattern_str;

		//前方車両がなければそのままの経路をpublish
		if(foreground_mobileye_.getSize() == 0 || stop_line_info.waypoint_index_ == -1)
		{
			std::cout << "abc," << foreground_mobileye_.getSize() << "," << stop_line_info.waypoint_index_<< std::endl;
			pub_waypoints_.publish(lane);

			autoware_msgs::CarCruiseStatus status;
			status.header.stamp = ros::Time::now();
			status.relative_velocity_mps = DBL_MIN;
			status.distance_x_m = -1;
			status.distance_rss_m = -1;
			status.tracking_mode = autoware_msgs::CarCruiseStatus::TRACKING_NO;
			status.accel_release = false;
			pub_car_cruise_status_.publish(status);

			track_pattern_str.data = "NO";
			prevTrakingAdd(autoware_msgs::CarCruiseStatus::TRACKING_NO);
			pub_track_pattern_.publish(track_pattern_str);
			return;
		}

		//最近傍の前方車両から車両追従に関するパラメータを計算する
		const autoware_msgs::TransformMobileyeObstacle foreground_obs = foreground_mobileye_.getLatestObs();//foreground_obstract_list_.getObs(foreground_obstract_list_.getSize()-1);
		const autoware_msgs::MobileyeCmdParam cmdacc = mathTargetAcc(foreground_mobileye_);

		autoware_msgs::Lane new_lane = *lane;
		autoware_msgs::Waypoint &first_waypoint = new_lane.waypoints[0];
		//double lineax = std::min(first_waypoint.twist.twist.linear.x, cmdacc.vehicle_ahead_velocity * std::pow((cmdacc.L0 / cmdacc.Lt),config_.avoid_collision_corr2) * config_.avoid_collision_corr1);//0.8);
		double lineax = std::max(0.0, cmdacc.vehicle_ahead_velocity * std::pow((cmdacc.L0 / cmdacc.Lt),config_.avoid_collision_corr2) * config_.avoid_collision_corr1);//0.8);
		std::stringstream ss;
		ss << lineax * 3.6 << ",";
		//first_waypoint.twist.twist.linear.x =std::min(first_waypoint.twist.twist.linear.x, lineax);
		ss << new_lane.waypoints[0].twist.twist.linear.x * 3.6 << "," << config_.avoid_collision_corr1 << "," << config_.avoid_collision_corr2;
		std_msgs::String way_vel;
		way_vel.data = ss.str();
		pub_waypoint_vel_.publish(way_vel);
		cmdPubnlish(cmdacc);

		//------------前方車両への追従方式の選定----------------
		//temporary_stopperに減速処理をさせるための停止線情報を設定
		//double distance = 0;
		double linea_x = 0;
		for(int waycou=0; waycou<new_lane.waypoints.size(); waycou++)
		{
			if(waycou==stop_line_info.waypoint_index_)
				linea_x = new_lane.waypoints[waycou].twist.twist.linear.x;
		}

		int prev_tracking = prev_tracking_mode_[prev_tracking_mode_.size()-1];
		int tracking = trackJudge(cmdacc, foreground_mobileye_, linea_x);//追跡タイプを計算

		autoware_msgs::CarCruiseStatus status;
		status.header.stamp = ros::Time::now();
		status.relative_velocity_mps = foreground_mobileye_.getObsRelativeVelMpsAve();
		status.distance_x_m = cmdacc.L0;
		status.distance_rss_m = cmdacc.Lt_RSS;
		status.expected_collision_time = cmdacc.expected_collision_time;
		status.tracking_mode = tracking;
		if(status.expected_collision_time >= 10.0 && status.expected_collision_time <= 20)
			status.accel_release = true; 
		else
			status.accel_release = false;
		pub_car_cruise_status_.publish(status);

		double waypoints_velocity_mps;
		int obs_index = -1;//前方車両が存在するwaypoint index
		std::cout << "stop_line_info.waypoint_index_," << stop_line_info.waypoint_index_ << std::endl; 
		for(int waycou=0; waycou<new_lane.waypoints.size(); waycou++)
		{
			autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[waycou];

			if(waycou==stop_line_info.waypoint_index_)
			{
				if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_STOP)
				{
					waypoints_velocity_mps = 0;
					obs_index = waycou;
				}
				else if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION1)
				{
					waypoints_velocity_mps = foreground_obs.velocity_mps;
					obs_index = waycou;
				}
				else if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION2)
				{
					waypoints_velocity_mps = foreground_obs.velocity_mps;
					obs_index = waycou;
				}
				else if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_EMG_DECELERATION)
				{
					waypoints_velocity_mps = 0;

					double v = current_velocity_ave_mps_;
					double a = 0.45;
					double L = v*v/(2*a);

					if(cmdacc.L0 > L)
					{
						tracking == autoware_msgs::CarCruiseStatus::TRACKING_NO;
						break;
					}

					double dis = 0;
					for(int i=1; i<new_lane.waypoints.size(); i++)
					{
						obs_index = new_lane.waypoints.size() - 1;

						geometry_msgs::Point po1 = new_lane.waypoints[i].pose.pose.position;
						geometry_msgs::Point po2 = new_lane.waypoints[i-1].pose.pose.position;
						dis += euclideanDistance(po1, po2);
						if(dis > L)
						{
							obs_index = i;
							break;
						}
					}
				}
				else if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_ACCELERATION)
				{
					waypoints_velocity_mps = foreground_mobileye_.getVelocityMpsAve();
					obs_index = waycou;
				}
				break;
			}
		}

		//確認用
		switch(tracking)
		{
		case autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION1:
			track_pattern_str.data = "DECELERATION1";
			break;
		case autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION2:
			track_pattern_str.data = "DECELERATION2";
			break;
		case autoware_msgs::CarCruiseStatus::TRACKING_STOP:
			track_pattern_str.data = "STOP";
			break;
		case autoware_msgs::CarCruiseStatus::TRACKING_EMG_DECELERATION:
			track_pattern_str.data = "EMG_DECELERATION";
			break;
		case autoware_msgs::CarCruiseStatus::TRACKING_ACCELERATION:
			track_pattern_str.data = "ACCELERATION";
			break;
		default:
			track_pattern_str.data = "NO";
		}
		pub_track_pattern_.publish(track_pattern_str);


		//------------前方車両への追従経路の作成----------------
		//指定速度通過線位置の設定
		double L1 = cmdacc.L0 / 2.0;
		//double distance_th = (cmdacc.L0 > cmdacc.Lt_prime) ? cmdacc.Lt_prime : L1;//前方車両までの距離と理想車間距離で停止線を引く位置を切り替える
		double distance_th = (cmdacc.L0 > cmdacc.Lt_RSS) ? cmdacc.Lt_RSS : L1;//前方車両までの距離と理想車間距離で停止線を引く位置を切り替える
		//std::string distance_th_str = (cmdacc.L0 > cmdacc.Lt_prime) ? "Lt_primu" : "L1";
		std::string distance_th_str = (cmdacc.L0 > cmdacc.Lt_RSS) ? "Lt_RSS" : "L1";

		//経路に停止線情報を設定
		if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION2)
		{
			double distance = 0;
			int waycou1;
			for(waycou1=obs_index-1; waycou1>=0; waycou1--)
			{
				autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[waycou1];
				geometry_msgs::Point po1 = new_lane.waypoints[waycou1].pose.pose.position;
				geometry_msgs::Point po2 = new_lane.waypoints[waycou1+1].pose.pose.position;
				distance += euclideanDistance(po1, po2);
				//curr_waypoint.twist.twist.linear.x = foreground_mobileye_.getVelocityMpsAve();
				//if(distance > cmdacc.Lt_prime)
				if(distance > cmdacc.Lt_RSS)
				{
					curr_waypoint.waypoint_param.object_stop_line = 1;
					curr_waypoint.waypoint_param.temporary_fixed_velocity_kmh = foreground_mobileye_.getVelocityMpsAve() * 3.6;
					curr_waypoint.waypoint_param.temporary_acceleration = cmdacc.cmd_acc;
					curr_waypoint.waypoint_param.velocity_limit_kmh = 100;
					break;
				}
			}
		}
		else if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_STOP
			|| tracking == autoware_msgs::CarCruiseStatus::TRACKING_EMG_DECELERATION)
		{
			int prev_tracking = prev_tracking_mode_[prev_tracking_mode_.size()-1];
			/*if(prev_tracking != autoware_msgs::CarCruiseStatus::TRACKING_STOP && prev_tracking !=  autoware_msgs::CarCruiseStatus::TRACKING_EMG_DECELERATION)
			{
				velocity_limit_of_stopmode_kmh_ = //current_velocity_ave_mps_ * 3.6 / 2.0;
				std::cout << "velocity_limit_of_stopmode_kmh," << velocity_limit_of_stopmode_kmh_ << std::endl;
			}*/
			velocity_limit_of_stopmode_kmh_ = 35;
			double bunsi = std::pow(current_velocity_ave_mps_, 2) - std::pow(cmdacc.vehicle_ahead_velocity, 2);
			double bunbo = 2 * stop_line_info.distance_;
			double dec_acc = bunsi / bunbo;
			std::cout << "v2," << cmdacc.vehicle_ahead_velocity << std::endl;
			std::cout << "v02," << current_velocity_ave_mps_ << std::endl;
			std::cout << "dis," << stop_line_info.distance_ << std::endl;
			std::cout << "dec_acc," << dec_acc << std::endl;
			//if(dec_acc < 0) dec_acc = 0;

			double distance = 0;
			//std::cout << "obs_index," << obs_index << std::endl;
			int waycou;
			for(waycou=obs_index-1; waycou>=0; waycou--)
			{
				autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[waycou];
				geometry_msgs::Point po1 = new_lane.waypoints[waycou].pose.pose.position;
				geometry_msgs::Point po2 = new_lane.waypoints[waycou+1].pose.pose.position;
				distance += euclideanDistance(po1, po2);
				//std::cout << "distance," << distance << "," << cmdacc.Lt_prime << std::endl;

				if(distance > distance_th)
				{
					std::cout << "tracking stop" << std::endl;
					curr_waypoint.waypoint_param.object_stop_line = 1;
					//curr_waypoint.waypoint_param.signal_stop_line = 1;
					//curr_waypoint.waypoint_param.temporary_deceleration = ;
					curr_waypoint.waypoint_param.temporary_fixed_velocity_kmh = 0;
					curr_waypoint.waypoint_param.temporary_acceleration = -0.30;//dec_acc;
					curr_waypoint.waypoint_param.velocity_limit_kmh = velocity_limit_of_stopmode_kmh_;//35;
					break;
				}
				else
				{
					//std::cout << "vel_ave," << waypoints_velocity_mps*3.6 << std::endl;
					//curr_waypoint.twist.twist.linear.x = waypoints_velocity_mps;
				}
			}

			//停止線を引く距離が自車両と前方車両までの距離を超えていた場合は、local waypoints 0番に停止線を置く
			if(waycou < 0)
			{
				autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[0];
				curr_waypoint.waypoint_param.object_stop_line = 1;
				//curr_waypoint.waypoint_param.temporary_deceleration = ;
				curr_waypoint.waypoint_param.temporary_fixed_velocity_kmh = 0;
				curr_waypoint.waypoint_param.temporary_acceleration = -0.30;//dec_acc;
				curr_waypoint.waypoint_param.velocity_limit_kmh = velocity_limit_of_stopmode_kmh_;//35;
			}
		}
		else if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_DECELERATION1)
		{
			//現在車間距離が理想車間距離よりも短い場合は、理想車間距離の半分の距離が理想車間距離となる速度を求める
			double delay_time = 1.0;
			double a = target_deceleration_ * 9.8;
			double adt = a * delay_time;//空想距離
			//double LL = L1 + cmdacc.Lt_prime - config_.min_free_running_distance;
			double LL = L1 + cmdacc.Lt_RSS - config_.min_free_running_distance;		
			//double v1 = -adt + std::sqrt(adt * adt + 2*a*L1);//L1を車間距離とする速度１
			//double v2 = std::sqrt(2*a * (L1-config_.min_free_running_distance));//L1を車間距離とする速度２
			double v1 = -adt + std::sqrt(adt * adt + 2*a*LL);//LLを車間距離とする速度１
			double v2 = std::sqrt(2*a * (LL-config_.min_free_running_distance));//LLを車間距離とする速度２
			double v3 = std::min(v1, v2);//L1を車間距離とする速度

			//double cmd_pass_vel =  (cmdacc.L0 > cmdacc.Lt_prime) ? waypoints_velocity_mps : v3;
			//std::string cmd_pass_vel_str = (cmdacc.L0 > cmdacc.Lt_prime) ? "waypoints_velocity_mps" : "v3";
			double cmd_pass_vel =  (cmdacc.L0 > cmdacc.Lt_RSS) ? waypoints_velocity_mps : v3;
			std::string cmd_pass_vel_str = (cmdacc.L0 > cmdacc.Lt_RSS) ? "waypoints_velocity_mps" : "v3";

			std::stringstream line_th_status_ss;
			line_th_status_ss << distance_th_str << "," << distance_th << "  " << cmd_pass_vel_str << "," << cmd_pass_vel;
			std_msgs::String line_th_status_msg;
			line_th_status_msg.data = line_th_status_ss.str();
			pub_line_th_status_.publish(line_th_status_msg);

			double distance = 0;
			//std::cout << "obs_index," << obs_index << std::endl;
			int waycou;
			for(waycou=obs_index-1; waycou>=0; waycou--)
			{
				autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[waycou];
				geometry_msgs::Point po1 = new_lane.waypoints[waycou].pose.pose.position;
				geometry_msgs::Point po2 = new_lane.waypoints[waycou+1].pose.pose.position;
				distance += euclideanDistance(po1, po2);
				//std::cout << "distance," << distance << "," << cmdacc.Lt_prime << std::endl;

				//if(distance > distance_th && tracking != TRACKING_ACCELERATION)
				if(distance > distance_th)
				{
					//if(tracking == TRACKING_DECELERATION1)
					{
						curr_waypoint.waypoint_param.object_stop_line = 1;
						//curr_waypoint.waypoint_param.temporary_deceleration = ;
						curr_waypoint.waypoint_param.temporary_fixed_velocity_kmh = cmd_pass_vel * 3.6;
						curr_waypoint.waypoint_param.velocity_limit_kmh = 100;
					}
					break;
				}
				/*else
				{
					//std::cout << "vel_ave," << waypoints_velocity_mps*3.6 << std::endl;
					curr_waypoint.twist.twist.linear.x = waypoints_velocity_mps;
				}*/
			}

			//停止線を引く距離が自車両と前方車両までの距離を超えていた場合は、local waypoints 0番に停止線を置く
			if(waycou < 0)
			{
				autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[0];
				curr_waypoint.waypoint_param.object_stop_line = 1;
				//curr_waypoint.waypoint_param.temporary_deceleration = ;
				curr_waypoint.waypoint_param.temporary_fixed_velocity_kmh = cmd_pass_vel * 3.6;
				curr_waypoint.waypoint_param.velocity_limit_kmh = 100;
			}
		}
		else if(tracking == autoware_msgs::CarCruiseStatus::TRACKING_ACCELERATION)
		{
			double distance = 0;
			int waycou;
			double Ldistnace =  cmdacc.L0 - cmdacc.Lt_RSS;//責任車間距離までの距離
			double kp = 1;//距離に関するPID_P;
			double vel_plus = Ldistnace * kp;
			//for(waycou=obs_index-1; waycou>=0; waycou--)
			for(waycou=0; waycou<new_lane.waypoints.size(); waycou++)
			{
				autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[waycou];
				std::cout << "vel_ave," << waypoints_velocity_mps*3.6 << std::endl;
				curr_waypoint.twist.twist.linear.x = std::max(waypoints_velocity_mps + vel_plus, 0.0);
			}
		}

		prevTrakingAdd(tracking);
		pub_waypoints_.publish(new_lane);
	}

	void callbackConfig(const autoware_config_msgs::ConfigMobileyeTracker::ConstPtr &msg)
	{
		config_ = *msg;
	}

	//
	void publishAccelCanCap(const StopLineInfo &stop_line_info)
	{
		std_msgs::Int16 accel_stroke_cap;
		if(foreground_mobileye_.getSize() == 0)
		{
			accel_stroke_cap.data = MAX_CAN_ACCEL_STROKE_CAP;
		}
		else
		{
			/*const autoware_msgs::TransformMobileyeObstacle obs = foreground_mobileye_.getLatestObs();
			if(obs.velocity_mps <= CAN_SEND_TH_KMH / 3.6
				&& obs.orig_data.object_accel_x <= CAN_SEND_TH_ACC
				&& stop_line_info.distance_ <= CAN_SEND_TH_DISTANCE)
			{
				accel_stroke_cap.data = MIN_CAN_ACCEL_STROKE_CAP;
			}
			else
				accel_stroke_cap.data = MAX_CAN_ACCEL_STROKE_CAP;*/
			const autoware_msgs::MobileyeCmdParam cmdacc = mathTargetAcc(foreground_mobileye_);
			if(cmdacc.L0 > cmdacc.Lt_RSS * LT_RSS_TH_MAGN)
				accel_stroke_cap.data = MAX_CAN_ACCEL_STROKE_CAP;
			else
				accel_stroke_cap.data = MIN_CAN_ACCEL_STROKE_CAP;
		}
		pub_can_stroke_cap_.publish(accel_stroke_cap);
	}

	void callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
	{
		StopLineInfo stop_line_info = searchMobileyeFollowing(msg);

		/*if(foreground_obstract_list_.getSize() > 0)//コマンド計算結果表示
		{
			const autoware_msgs::MobileyeCmdParam cmdacc = mathTargetAcc(foreground_obstract_list_);
			cmdPubnlish(cmdacc);
		}*/
		createCarTrackingWaypoints(msg, stop_line_info);
		publishAccelCanCap(stop_line_info);
	}

	void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		current_pose_ = *msg;
	}

	void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		current_velocity_list_.push_back(*msg);
		if(current_velocity_list_.size() > CURRENT_VELOCITY_LIST_SIZE)
			current_velocity_list_.erase(current_velocity_list_.begin());

		double vel = mathVelAve();

		if(current_velocity_list_.size() >= 2)
		{
			ros::Duration ros_time_diff =
				current_velocity_list_[current_velocity_list_.size()-1].header.stamp - current_velocity_list_[current_velocity_list_.size()-2].header.stamp;
			double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
			current_acc_ave_mps_ = (vel - current_velocity_ave_mps_) / time_diff;
		}
		else
		{
			current_acc_ave_mps_ = 0;
		}
		current_velocity_ave_mps_ = vel;
		//current_velocity_ = *msg;
	}

	//mobileye車両オブジェクトを追加
	void addMobileyeObstract(const autoware_msgs::TransformMobileyeObstacle::ConstPtr &msg)
	{
		bool add_flag = false;
		//for(size_t info_cou=0; info_cou<mobileye_obstract_info_list_.size(); info_cou++)
		for(MobileyeObstacleInfo &obs_list : mobileye_obstacle_info_list_)
		{
			//MobileyeObstractInfo &obs_list = mobileye_obstract_info_list_[info_cou];
			const autoware_msgs::TransformMobileyeObstacle &obs = obs_list.getObs(0);
			if(obs.orig_data.obstacle_id == msg->orig_data.obstacle_id)//オブジェクトIDが同一なら追加
			{
				obs_list.push(*msg);
				add_flag = true;
			}
		}

		if(add_flag == false)//オブジェクトIDが既存になければ追加
		{
			MobileyeObstacleInfo new_info;
			new_info.push(*msg);
			mobileye_obstacle_info_list_.push_back(new_info);
		}
	}

	//前方車両がwaypoint範囲にあるか(前方車両判定)
	//cleft : waypoint範囲左
	//cright : waypoint範囲右
	//fleft : 前方車両左
	//fright : 前方車両右
	bool judgFollowing(const double cleft, const double cright, const double fleft, const double fright)
	{
		if(cright < fright)//waypoint範囲右が前方車両右より内側
		{
			if(cright > fleft)//waypoint範囲右が前方車両左より外側
				return true;
		}
		else if(cleft > fleft)//waypoint範囲左が前方車両左より内側
		{
			if(cleft < fright)//waypoint範囲左が前方車両右より外側
				return true;
		}
		else if(cright > fright && cleft < fleft)//前方車両左右がwaypoint範囲左右内ある
			return true;

		return false;
	}

	//最近傍の正面mobileyeを探索
	StopLineInfo searchMobileyeFollowing(const autoware_msgs::Lane::ConstPtr &lane)
	{
		int hit_obs_index = -1; //判定車両ID
		int hit_waypoint_index = -1; //判定waypoint
		double x_adj = 0; //判定waypointのx微調整位置

		int wc;
		double front_dis_sum = 0;
		double front_fraction = 0;//wcのwaypoint位置から見た車両フロントの距離
		for(wc=1; wc<lane->waypoints.size(); wc++)
		{
			const autoware_msgs::Waypoint waypoint = lane->waypoints[wc];
			const autoware_msgs::Waypoint prev_waypoint = lane->waypoints[wc-1];
			const geometry_msgs::Point way_po = waypoint.pose.pose.position;
			const geometry_msgs::Point prev_way_po = prev_waypoint.pose.pose.position;

			front_dis_sum += euclideanDistance(way_po, prev_way_po);
			if(front_dis_sum > front_bumper_to_baselink_ + mobileye_virtual_bumper_)
			{
				front_fraction = front_dis_sum - front_bumper_to_baselink_ - mobileye_virtual_bumper_;
				break;
			}
		}

		//経路上の正面車両を探索
		for(int waycou=wc; waycou<lane->waypoints.size(); waycou++)
		{
			const autoware_msgs::Waypoint waypoint = lane->waypoints[waycou];
			const autoware_msgs::Waypoint prev_waypoint = lane->waypoints[waycou-1];
			const geometry_msgs::Point way_po = waypoint.pose.pose.position;
			const geometry_msgs::Point prev_way_po = prev_waypoint.pose.pose.position;

			//経路と自車両のy距離を計算
			double y_dis = y_distanbce(way_po, prev_way_po, current_pose_.pose.position);
			//std::cout << "y_dis," << y_dis << "," << way_po.x << "," << prev_way_po.x << "," << current_pose_.pose.position.x << std::endl;

			for(int obscou=0; obscou<mobileye_obstacle_info_list_.size(); obscou++)
			{
				const MobileyeObstacleInfo &obs_info = mobileye_obstacle_info_list_[obscou];
				autoware_msgs::TransformMobileyeObstacle latest_obs = obs_info.getLatestObs();
				geometry_msgs::Point obs_po = latest_obs.map_pose.position;
				
				Eigen::Vector3d way_diff_po(way_po.x-prev_way_po.x, way_po.y-prev_way_po.y, way_po.z-prev_way_po.z);
				Eigen::Vector3d obs_diff_po(obs_po.x-prev_way_po.x, obs_po.y-prev_way_po.y, obs_po.z-prev_way_po.z);
				Eigen::AngleAxisd way_yaw(std::atan2(way_diff_po.y(), way_diff_po.x()), Eigen::Vector3d::UnitZ());
				Eigen::Quaterniond rot_yaw(way_yaw.inverse());
				Eigen::Vector3d way_diff_po_rot_yaw = rot_yaw * way_diff_po;
				Eigen::Vector3d obs_diff_po_rot_yaw = rot_yaw * obs_diff_po;
				Eigen::AngleAxisd way_pitch(std::atan2(way_diff_po_rot_yaw.z(), way_diff_po_rot_yaw.x()), Eigen::Vector3d::UnitY());
				Eigen::Quaterniond rot_pitch(way_pitch.inverse());
				Eigen::Vector3d way_diff_po_rot = rot_pitch * way_diff_po_rot_yaw;
				Eigen::Vector3d obs_diff_po_rot = rot_pitch * obs_diff_po_rot_yaw;

				double cright = way_diff_po_rot.y() + vehicle_width_/2;
				double cleft = way_diff_po_rot.y() - vehicle_width_/2;
				double fright = obs_diff_po_rot.y() + obs_info.getLatestObs().orig_data.obstacle_width/2;
				double fleft = obs_diff_po_rot.y() - obs_info.getLatestObs().orig_data.obstacle_width/2;
				//std::cout << "id," << obs_info.getLatestObs().orig_data.obstacle_id << " way_y," << way_diff_po_rot.y() << " obs_y," << obs_diff_po_rot.y() << " vehicle_w," << vehicle_width_/2 << " obs_w," << obs_info.getLatestObs().orig_data.obstacle_width/2 << std::endl;
				//std::cout << "cleft," << cleft << " cright," << cright << " fleft," << fleft << " fright," << fright << std::endl;
				if(judgFollowing(cleft, cright, fleft, fright)
					&& obs_diff_po_rot.x() >= 0
					&& obs_diff_po_rot.x() <= way_diff_po_rot.x())
				{
					//std::cout << "follwing," << cleft << "," << cright << "," << fleft << "," << fright << "," << y_dis << std::endl;
					hit_obs_index = obscou;
					hit_waypoint_index = waycou;
					x_adj = way_diff_po_rot.x() - obs_diff_po_rot.x();
				}
			}

			if(hit_obs_index != -1) break;
		}

		//正面車両が存在した場合は停止線情報を設定
		std::cout << "hit_mobileye," << hit_obs_index << std::endl;
		if(hit_obs_index != -1)
		{
			double dis_sum = x_adj;
			double dis_adj = 0;
			int stop_index = 0;
			bool setflag = false;

			for(int waycou=hit_waypoint_index; waycou>=1; waycou--)
			{
				const autoware_msgs::Waypoint waypoint = lane->waypoints[waycou];
				const autoware_msgs::Waypoint prev_waypoint = lane->waypoints[waycou-1];
				const geometry_msgs::Point way_po = waypoint.pose.pose.position;
				const geometry_msgs::Point prev_way_po = prev_waypoint.pose.pose.position;
				double euc_distance = euclideanDistance(way_po, prev_way_po);
				dis_sum += euc_distance;
				if(dis_sum >= config_.min_free_running_distance && setflag == false)
				{
					dis_adj = dis_sum - config_.min_free_running_distance;
					stop_index = waycou;
					setflag = true;
					//break;
				}
			}

			foreground_mobileye_ = mobileye_obstacle_info_list_[hit_obs_index];
			StopLineInfo info;
			info.waypoint_index_ = stop_index;
			info.x_adj_ = dis_adj;
			info.distance_ = dis_sum - front_bumper_to_baselink_ - mobileye_virtual_bumper_;
			pub_front_mobileye_obstacle_.publish(foreground_mobileye_.getLatestObs());

			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();
			marker.ns = "front_car_mobileye";
			marker.id = 0;

			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.lifetime = ros::Duration();

			marker.scale.x = 2;
			marker.scale.y = 2;
			marker.scale.z = 2;
			autoware_msgs::TransformMobileyeObstacle front_car = foreground_mobileye_.getLatestObs();
			marker.pose.position.x = front_car.map_pose.position.x;
			marker.pose.position.y = front_car.map_pose.position.y;
			marker.pose.position.z = front_car.map_pose.position.z;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;
			marker.pose.orientation.z = 0;
			marker.pose.orientation.w = 1;
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0f;
			pub_front_car_marker_.publish(marker);
			return info;
		}
		else
		{
			foreground_mobileye_.clear();
			StopLineInfo info;
			info.waypoint_index_ = -1;
			info.x_adj_ = -1;
			info.distance_ = -1;
			autoware_msgs::TransformMobileyeObstacle msg;
			msg.orig_data.obstacle_id = USHRT_MAX;
			pub_front_mobileye_obstacle_.publish(msg);

			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();
			marker.ns = "front_car_mobileye";
			marker.id = 0;

			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::DELETE;
			marker.lifetime = ros::Duration();

			/*marker.scale.x = 2;
			marker.scale.y = 2;
			marker.scale.z = 2;
			autoware_msgs::TransformMobileyeObstacle front_car = foreground_mobileye_.getLatestObs();
			marker.pose.position.x = front_car.map_pose.position.x;
			marker.pose.position.y = front_car.map_pose.position.y;
			marker.pose.position.z = front_car.map_pose.position.z;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;
			marker.pose.orientation.z = 0;
			marker.pose.orientation.w = 1;
			marker.color.r = 1.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0f;*/
			pub_front_car_marker_.publish(marker);
	
			return info;
		}
	}

	void cmdPubnlish(const autoware_msgs::MobileyeCmdParam &cmd)
	{
		autoware_msgs::MobileyeCmdParam pub = cmd;
		pub.header.stamp = ros::Time::now();
		pub_mobileye_cmd_param_.publish(pub);
	}

	//mobileyeからの車両情報を受け取るコールバック
	void callbackMobileyeObstacle(const autoware_msgs::TransformMobileyeObstacle::ConstPtr &msg)
	{
		addMobileyeObstract(msg);
	}

	//正面判定された車両情報がconfig_.obs_delete_time秒間更新が無かった場合、その車両情報を消去する
	void callbackObsDeleteTimer(const ros::TimerEvent &e)
	{
		ros::Time ros_nowtime = ros::Time::now();//e.current_real;
		for(long i=mobileye_obstacle_info_list_.size()-1; i>=0; i--)
		{
			MobileyeObstacleInfo obs_info = mobileye_obstacle_info_list_[i];
			autoware_msgs::TransformMobileyeObstacle obs = obs_info.getLatestObs();//obs_info.getObs(obs_info.getSize()-1);
			ros::Duration ros_time_diff = ros_nowtime - obs.header.stamp;
			double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
			if(time_diff > config_.obs_delete_time) mobileye_obstacle_info_list_.erase(mobileye_obstacle_info_list_.begin() + i);
		}
	}

	void callbackTargetDeceleration(const std_msgs::Float64::ConstPtr &msg)
	{
		target_deceleration_ = msg->data;
	}
public:
	CarTrackinig(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, current_velocity_ave_mps_(0)
		, current_acc_ave_mps_(0)
		, velocity0_timer_(ros::Time(0))
		, target_deceleration_(0.1)
		, velocity_limit_of_stopmode_kmh_(0)
	{
		front_bumper_to_baselink_ = nh_.param<double>("/vehicle_info/front_bumper_to_baselink", 4.55);
		mobileye_virtual_bumper_ = nh_.param<double>("/mobileye/virtual_bumper", 2.0);
		vehicle_width_ = nh_.param<double>("/vehicle_info/vehicle_width", 2.3);

		sub_config_ = nh_.subscribe("/config/mobileye_tracker", 10, &CarTrackinig::callbackConfig, this);
		sub_waypoints_ = nh_.subscribe("/position_adjustment_waypoints", 10, &CarTrackinig::callbackWaypoints, this);
		sub_current_pose_ = nh_.subscribe("/current_pose", 10, &CarTrackinig::callbackCurrentPose, this);
		sub_current_velocity_ = nh_.subscribe("/current_velocity", 10, &CarTrackinig::callbackCurrentVelocity, this);
		sub_mobileye_obstacle_ = nh_.subscribe("/transform_mobileye_obstacle", 10, &CarTrackinig::callbackMobileyeObstacle, this);
		sub_target_deceleration_ = nh_.subscribe("/car_target_deceleration", 10, &CarTrackinig::callbackTargetDeceleration, this);

		pub_waypoints_ = nh_.advertise<autoware_msgs::Lane>("/car_tracking_waypoints", 10);
		pub_mobileye_cmd_param_ = nh_.advertise<autoware_msgs::MobileyeCmdParam>("/mobileye_tracker/cmd_param", 10);
		pub_waypoint_vel_ = nh_.advertise<std_msgs::String>("/mobileye_tracker/waypoint_vel", 10);
		pub_track_pattern_ = nh_.advertise<std_msgs::String>("/mobileye_tracker/track_pattern", 10);
		pub_front_mobileye_obstacle_ = nh_.advertise<autoware_msgs::TransformMobileyeObstacle>("/mobileye_tracker/front_mobileye", 10);
		pub_line_th_status_ = nh_.advertise<std_msgs::String>("/mobileye_tracker/line_th_status", 10);
		pub_can_stroke_cap_ = nh_.advertise<std_msgs::Int16>("/mobileye_tracker/accel_stroke_cap", 10);
		pub_car_cruise_status_ = nh_.advertise<autoware_msgs::CarCruiseStatus>("/car_cruise_status", 10);
		pub_front_car_marker_ = nh_.advertise<visualization_msgs::Marker>("/front_car_marker", 10);

		obs_delete_timer_ = nh_.createTimer(ros::Duration(0.1), &CarTrackinig::callbackObsDeleteTimer, this);

		//current_velocity_.twist.linear.x = 0;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mobileye_tracker");
	ros::NodeHandle nh, pnh("~");

	CarTrackinig tracking(nh, pnh);
	ros::spin();
	return 0;
}