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
#include <autoware_config_msgs/ConfigMobileyeTracker.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/TransformMobileyeObstacle.h>
#include <autoware_msgs/MobileyeCmdParam.h>
#include <autoware_msgs/CarCruiseStatus.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

double euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
	double x1 = p1.x,  x2 = p2.x;
	double y1 = p1.y,  y2 = p2.y;
	double z1 = p1.z,  z2 = p2.z;
	double xd = x1 - x2,  yd = y1 - y2,  zd = z1 - z2;
	return std::sqrt(xd*xd + yd*yd + zd*zd);
}

/*double mathAcc(const std::vector<geometry_msgs::TwistStamped> &vel_list)
{
	if(vel_list.size() <= 2) return 0;
	double sum = 0;
	for(size_t i=1; i<vel_list.size()-1; i++)
	{
		const geometry_msgs::TwistStamped &twist1 = vel_list[i-1];
		const geometry_msgs::TwistStamped &twist2 = vel_list[i+1];
		double vel_diff = twist2.twist.linear.x - twist1.twist.linear.x;
		ros::Duration ros_time_diff = twist2.header.stamp - twist1.header.stamp;
		double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
		sum += vel_diff / time_diff;
	}
	return sum / (vel_list.size()-2);
}*/

double mathAcc(const std::vector<autoware_msgs::TransformMobileyeObstacle> &vel_list)
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
}

//自車と先行車両の車間距離の計算
//msg : 指定車両の速度
//target_deceleration : 目標減速度
//delay_time : 前方車両を感知してからブレーキがかかるまでの距離
//min_free_running_distance : 空想距離
double targetFreeSpace(const double mps, const double target_deceleration, const double delay_time, const double min_free_running_distance)
{
	//double mps = kmh / 3.6;
	double free_running_distance = mps * delay_time;
	double brakeing_dinstace = mps * mps / (2 * target_deceleration * 9.8);
	return std::max(min_free_running_distance, free_running_distance) + brakeing_dinstace;
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

//同じ車両IDのデータを纏めて管轄するクラス
class ObstractInfo
{
public:
	const static size_t FORREGROUND_VELOCITY_LIST_SIZE = 20; //!< 前方車両速度listの最大サイズ
private:
	std::vector<autoware_msgs::TransformMobileyeObstacle> obs_list_; //!< 前方車両のリスト
	double obs_velocity_ave_; //!< 前方車両速度の平均
	double obs_relative_vel_ave_;
	double obs_relative_acc_ave_; 
	double obs_acc_ave_; //!< 前方車両加速度の平均
	double obs_pos_x_ave_;

	double mathVelAve()
	{
		double vel_ave_ = 0;
		obs_relative_vel_ave_ = 0;
		obs_relative_acc_ave_ = 0;
		obs_pos_x_ave_ = 0;

		for(const autoware_msgs::TransformMobileyeObstacle &obs : obs_list_)
		{
			vel_ave_ += obs.velocity_mps;
			obs_relative_vel_ave_ += obs.orig_data.obstacle_rel_vel_x;
			obs_relative_acc_ave_ += obs.orig_data.object_accel_x;
			obs_pos_x_ave_ += obs.orig_data.obstacle_pos_x;
		}
		obs_relative_vel_ave_ /= obs_list_.size();
		obs_relative_acc_ave_ /= obs_list_.size();
		obs_pos_x_ave_ /= obs_list_.size();
		return vel_ave_ / obs_list_.size();
		/*std::vector<double> list;
		for(const autoware_msgs::TransformMobileyeObstacle &obs : obs_list_)
			list.push_back(obs.velocity);
		std::sort(list.begin(), list.end());
		return list[list.size() / 2];*/
	}
public:
	ObstractInfo()
		: obs_velocity_ave_(0)
		, obs_acc_ave_(0)
		, obs_relative_vel_ave_(0)
		, obs_relative_acc_ave_(0)
		, obs_pos_x_ave_(0)
	{
	}

	void push(const autoware_msgs::TransformMobileyeObstacle &obs)
	{
		obs_list_.push_back(obs);
		if(obs_list_.size() > FORREGROUND_VELOCITY_LIST_SIZE)
			obs_list_.erase(obs_list_.begin());

		double vel = mathVelAve();

		if(obs_list_.size() >= 2)
		{
			ros::Duration ros_time_diff =
				obs_list_[obs_list_.size()-1].header.stamp - obs_list_[obs_list_.size()-2].header.stamp;
			double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
			obs_acc_ave_ = (vel - obs_velocity_ave_) / time_diff;
		}
		else
		{
			obs_acc_ave_ = 0;
		}
		obs_velocity_ave_ = vel;
	}

	size_t getSize() const
	{
		return obs_list_.size();
	}

	autoware_msgs::TransformMobileyeObstacle getObs(const int index) const
	{
		return obs_list_[index];
	}

	autoware_msgs::TransformMobileyeObstacle getLatestObs() const
	{
		return obs_list_[obs_list_.size()-1];
	}

	double getVelocityAve() const
	{
		return obs_velocity_ave_;
	}

	double getAccAve() const
	{
		return obs_acc_ave_;
	}

	double getObsRelativeVelAve() const
	{
		return obs_relative_vel_ave_;
	}

	double getObsRelativeAccAve() const
	{
		return obs_relative_acc_ave_;
	}

	double getPoseXAve() const
	{
		return obs_pos_x_ave_;
	}

	void clear()
	{
		obs_list_.clear();
		obs_velocity_ave_ = 0; //!< 前方車両速度の平均
		obs_relative_vel_ave_ = 0;
		obs_relative_acc_ave_ = 0; 
		obs_acc_ave_ = 0; //!< 前方車両加速度の平均
		obs_pos_x_ave_ = 0;
	}
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
	const static double CAN_SEND_TH_KMH = 8.0;//!<アクセルSTROKEキャップ通知の前方車両速度に対するしきい値(時速)
	const static double CAN_SEND_TH_ACC = 0.3;//!<アクセルSTROKEキャップ通知の相対加速度に対するしきい値(m/s2)
	const static double CAN_SEND_TH_DISTANCE = 40.0;//!<アクセルSTROKEキャップ通知の前方車両距離に対するしきい値(m)

	//const static double min_free_running_distance_ = 2.0; //!< 最小車間距離
	//const static double target_deceleration_ = 0.3; //!<目標減速度
	//const static double control_cycle_ = 10.0; //!<制御周期
	//const static double delay_time_ = 1.0 / control_cycle_ * 2.0; //!< 1/制御周期 * 2

	const static int TRACKING_NO = 0;
	const static int TRACKING_DECELERATION = 1;
	const static int TRACKING_EMG_DECELERATION = 2;
	const static int TRACKING_STOP = 3;
	const static int TRACKING_ACCELERATION = 4;

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber sub_config_;
	ros::Subscriber sub_waypoints_;
	ros::Subscriber sub_current_velocity_;
	ros::Subscriber sub_mobileye_obstacle_;
	ros::Subscriber sub_target_deceleration_;//!<目標減速度

	ros::Publisher pub_waypoints_;
	ros::Publisher pub_mobileye_cmd_param_;//!<速度計算確認用
	ros::Publisher pub_waypoint_vel_;
	ros::Publisher pub_track_pattern_;//!<前方車両に対する動作フラグをpublish
	ros::Publisher pub_front_mobileye_obstacle_;//!<前方車両と判断されたオブジェクト情報
	ros::Publisher pub_line_th_status_;
	ros::Publisher pub_can_stroke_cap_;//!<CANに通知するアクセルSTROKEキャップ
	ros::Publisher pub_car_cruise_status_;//!<前方車両追従の情報

	ros::Timer obs_delete_timer_;

	autoware_config_msgs::ConfigMobileyeTracker config_;
	ObstractInfo foreground_obstract_list_; //!< 探索範囲内で見つかった正面オブジェクト
	//geometry_msgs::TwistStamped current_velocity_; //!< 現在の自車両速度
	std::vector<geometry_msgs::TwistStamped> current_velocity_list_; //!< current_velocityのリスト
	double current_velocity_ave_; //!< 現在の自車両速度
	double current_acc_ave_; //!<現在の加速度
	std::vector<ObstractInfo> obstract_info_list_; //!<mobileyeからの車両情報のリスト
	ros::Time velocity0_timer_; //!<障害物速度が最初に0と判断された時間
	double front_bumper_to_baselink_; //!< baselinkからフロントバンパーまでの距離
	geometry_msgs::PoseStamped current_pose_;
	double target_deceleration_;//!<目標減速度

	double mathVelAve()
	{
		double sum = 0;
		for(const geometry_msgs::TwistStamped &twist : current_velocity_list_)
			sum += twist.twist.linear.x;
		return sum / (current_velocity_list_.size());
	}

	//autoware_msgs::MobileyeCmdParam mathTargetAcc(const autoware_msgs::TransformMobileyeObstacle obs) const
	autoware_msgs::MobileyeCmdParam mathTargetAcc(const ObstractInfo &obs_list) const
	{
		//const autoware_msgs::TransformMobileyeObstacle &foreground_obs = foreground_obstract_[foreground_obstract_.size()-1];
		const autoware_msgs::TransformMobileyeObstacle obs = obs_list.getLatestObs();//obs_list.getObs(obs_list.getSize()-1);

		double relative_vel = obs_list.getObsRelativeVelAve();//obs.orig_data.obstacle_rel_vel_x;//前方車両と自車両との相対速度
		double relative_acc = obs_list.getObsRelativeAccAve();//obs.orig_data.object_accel_x;//前方車両と自車両との相対加速度
		double current_vel = current_velocity_ave_;//current_velocity_.twist.linear.x;
		double current_acc = current_acc_ave_;//mathAcc(current_velocity_list_);//自車両の加速度
		
		double vehicle_ahead_vel = obs_list.getVelocityAve();//relative_vel + current_vel;//前方車両の速度
		double vehicle_ahead_acc = obs_list.getAccAve();//relative_acc + current_acc;//前方車両の加速度
		double vehicle_ahead_acc2 = 0;//mathAcc(foreground_obstract_list_);
		//double Lt = targetFreeSpace(current_velocity_.twist.linear.x, target_deceleration_, delay_time_, min_free_running_distance_);
		double delay_time = 1.0 / config_.control_cycle * 2.0;
		double Lt = targetFreeSpace(current_velocity_ave_, target_deceleration_, delay_time, config_.min_free_running_distance);
		double Lt_prime = targetFreeSpace(vehicle_ahead_vel, target_deceleration_, delay_time, config_.min_free_running_distance);
		double L0_Lt = targetFreeSpace(vehicle_ahead_vel, MAX_DECELERATION, delay_time, config_.min_free_running_distance);
		double L0 = obs_list.getPoseXAve();//obs.orig_data.obstacle_pos_x;//前方車両までの距離
		double cmd_acc = vehicle_ahead_acc + (1.0 / 2.0) * std::pow(relative_vel, 2.0) / (Lt - L0);
		double Lt_part1 = (vehicle_ahead_vel-current_vel)*(-(vehicle_ahead_vel-current_vel)/(vehicle_ahead_acc-cmd_acc));
		double Lt_part2 = 0.5*(vehicle_ahead_acc-cmd_acc)*std::pow(-(vehicle_ahead_vel-current_vel)/(vehicle_ahead_acc-cmd_acc),2.0);
		double Lt2 = L0 + Lt_part1 + Lt_part2;
		double target_scheduled_achievement_time = 2.0 * (Lt - L0) / (relative_vel);
		double expected_collision_time;// = (relative_vel == 0) ? 10000.0 : L0 / (-relative_vel);
		if(relative_vel >= 0) expected_collision_time = DBL_MAX;//前方車両が近づかない場合は衝突予想時間を最大値にする
		else expected_collision_time = L0 / (-relative_vel);

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
		cmdacc.current_velocity = current_velocity_ave_;//current_vel;
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


	int trackJudge(const autoware_msgs::MobileyeCmdParam &cmd, const autoware_msgs::TransformMobileyeObstacle &obs, const double way_vel)
	{
		int ret = TRACKING_NO;

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
		if(obs.velocity_mps < 5.0 / 3.6 || cmd.expected_collision_time < 10.0//cmd.relative_vel < MAX_ACCEPTABLE_REL_VEL / 3.6
			|| obs.orig_data.obstacle_status == mobileye_560_660_msgs::ObstacleData::OBSTACLE_STATUS_STOPPED
			|| obs.orig_data.obstacle_status == mobileye_560_660_msgs::ObstacleData::OBSTACLE_STATUS_PARKED
			|| obs.orig_data.obstacle_status == mobileye_560_660_msgs::ObstacleData::OBSTACLE_STATUS_STANDING)//停止
		{
			ros::Time nowtime = ros::Time::now();
			if(velocity0_timer_ == ros::Time(0)) velocity0_timer_ = nowtime;
			ros::Duration ros_time_diff = nowtime - velocity0_timer_;
			double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
			if(time_diff >= 0.1) ret = TRACKING_STOP;
		}
		else
		{
			velocity0_timer_ = ros::Time(0);

			//if(cmd.Lt > cmd.L0 && cmd.Lt_prime < cmd.L0 && cmd.relative_vel < 0)
			if(cmd.Lt > cmd.L0 && cmd.relative_vel < 0)
			{
				ret = TRACKING_DECELERATION;//減速
			}
			//else if(cmd.L0 < cmd.Lt_prime && cmd.relative_vel < 0)
			//else if((cmd.L0 < cmd.L0_Lt || cmd.expected_collision_time < 3.0) && cmd.relative_vel < 0)
			else if(cmd.expected_collision_time < 5.0)
			{
				ret = TRACKING_EMG_DECELERATION;//緊急減速
			}
			else if(cmd.relative_vel > 0 && cmd.vehicle_ahead_velocity < way_vel)
			{
				ret = TRACKING_ACCELERATION;//前方車両の速度に合わせる
			}
		}

		return ret;
	}

	//前方車両速度から追跡経路の速度を作成
	void createCarTrackingWaypoints(const autoware_msgs::Lane::ConstPtr &lane, const StopLineInfo &stop_line_info)
	{
		//前方車両がなければそのままの経路をpublish
		//std::cout << "foreground_obstract_list_," << foreground_obstract_list_.getSize() << std::endl;
		if(foreground_obstract_list_.getSize() == 0 || stop_line_info.waypoint_index_ == -1)
		{
			std::cout << "abc," << foreground_obstract_list_.getSize() << "," << stop_line_info.waypoint_index_<< std::endl;
			pub_waypoints_.publish(lane);

			autoware_msgs::CarCruiseStatus status;
			status.header.stamp = ros::Time::now();
			status.relative_velocity_mps = DBL_MIN;
			status.distance_x_m = -1;
			pub_car_cruise_status_.publish(status);

			return;
		}

		//最近傍の前方車両から車両追従に関するパラメータを計算する
		const autoware_msgs::TransformMobileyeObstacle foreground_obs = foreground_obstract_list_.getLatestObs();//foreground_obstract_list_.getObs(foreground_obstract_list_.getSize()-1);
		const autoware_msgs::MobileyeCmdParam cmdacc = mathTargetAcc(foreground_obstract_list_);

		autoware_msgs::CarCruiseStatus status;
		status.header.stamp = ros::Time::now();
		status.relative_velocity_mps = foreground_obstract_list_.getObsRelativeVelAve();
		status.distance_x_m = cmdacc.L0;
		status.expected_collision_time = cmdacc.expected_collision_time;
		pub_car_cruise_status_.publish(status);

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

		//temporary_stopperに減速処理をさせるための停止線情報を設定
		//double distance = 0;
		double linea_x = 0;
		for(int waycou=0; waycou<new_lane.waypoints.size(); waycou++)
		{
			if(waycou==stop_line_info.waypoint_index_)
				linea_x = new_lane.waypoints[waycou].twist.twist.linear.x;
		}
		int tracking = trackJudge(cmdacc, foreground_obs, linea_x);//追跡タイプを計算
		double waypoints_velocity_mps;
		int obs_index = -1;//前方車両が存在するwaypoint index
		std::cout << "stop_line_info.waypoint_index_," << stop_line_info.waypoint_index_ << std::endl; 
		for(int waycou=0; waycou<new_lane.waypoints.size(); waycou++)
		{
			autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[waycou];

			if(waycou==stop_line_info.waypoint_index_)
			{
				if(tracking == TRACKING_STOP)
				{
					waypoints_velocity_mps = 0;
					obs_index = waycou;
				}
				else if(tracking == TRACKING_DECELERATION)
				{
					waypoints_velocity_mps = foreground_obs.velocity_mps;
					obs_index = waycou;
				}
				else if(tracking == TRACKING_EMG_DECELERATION)
				{
					waypoints_velocity_mps = 0;

					double v = current_velocity_ave_;
					double a = 0.45;
					double L = v*v/(2*a);

					if(cmdacc.L0 > L)
					{
						tracking == TRACKING_NO;
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
				else if(tracking == TRACKING_ACCELERATION)
				{
					waypoints_velocity_mps = foreground_obstract_list_.getVelocityAve();
					obs_index = waycou;
				}
				break;
			}
		}

		//確認用
		std_msgs::String track_pattern_str;
		switch(tracking)
		{
		case TRACKING_DECELERATION:
			track_pattern_str.data = "TRACKING_DECELERATION";
			break;
		case TRACKING_STOP:
			track_pattern_str.data = "TRACKING_STOP";
			break;
		case TRACKING_EMG_DECELERATION:
			track_pattern_str.data = "TRACKING_EMG_DECELERATION";
			break;
		case TRACKING_ACCELERATION:
			track_pattern_str.data = "TRACKING_ACCELERATION";
			break;
		default:
			track_pattern_str.data = "TRACKING_NO";
		}
		pub_track_pattern_.publish(track_pattern_str);

		//経路に停止線情報を設定
		if(tracking == TRACKING_DECELERATION || tracking == TRACKING_STOP || tracking == TRACKING_EMG_DECELERATION || tracking == TRACKING_ACCELERATION)
		{
			double distance = 0;
			//std::cout << "obs_index," << obs_index << std::endl;
			for(int waycou=obs_index-1; waycou>=0; waycou--)
			{
				autoware_msgs::Waypoint &curr_waypoint = new_lane.waypoints[waycou];
				geometry_msgs::Point po1 = new_lane.waypoints[waycou].pose.pose.position;
				geometry_msgs::Point po2 = new_lane.waypoints[waycou+1].pose.pose.position;
				distance += euclideanDistance(po1, po2);
				//std::cout << "distance," << distance << "," << cmdacc.Lt_prime << std::endl;

				//指定速度通過線位置の設定
				double L1 = cmdacc.L0 / 2.0;
				double distance_th = (cmdacc.L0 > cmdacc.Lt_prime) ? cmdacc.Lt_prime : L1;//前方車両までの距離と理想車間距離で停止線を引く位置を切り替える
				std::string distance_th_str = (cmdacc.L0 > cmdacc.Lt_prime) ? "Lt_primu" : "L1";

				//指定速度停止速度の設定
				double a = target_deceleration_ * 9.8;
				double free_distance = a * config_.obs_delete_time;//空想距離
				double v1 = -free_distance + std::sqrt(free_distance * free_distance + 2*a*L1);//L1を車間距離とする速度１
				double v2 = std::sqrt(2*a * (L1-config_.min_free_running_distance));//L1を車間距離とする速度２
				double v3 = std::min(v1, v2);//L1を車間距離とする速度
				double cmd_pass_vel =  (cmdacc.L0 > cmdacc.Lt_prime) ? waypoints_velocity_mps : v3;
				std::string cmd_pass_vel_str = (cmdacc.L0 > cmdacc.Lt_prime) ? "waypoints_velocity_mps" : "v3";

				if(distance > distance_th && tracking != TRACKING_ACCELERATION)
				{
					if(tracking == TRACKING_DECELERATION)
					{
						curr_waypoint.waypoint_param.object_stop_line = 1;
						//curr_waypoint.waypoint_param.temporary_deceleration = ;
						curr_waypoint.waypoint_param.temporary_fixed_velocity = cmd_pass_vel * 3.6;
					}
					else if(tracking == TRACKING_STOP || tracking == TRACKING_EMG_DECELERATION)
					{
						std::cout << "tracking stop" << std::endl;
						curr_waypoint.waypoint_param.object_stop_line = 1;
						//curr_waypoint.waypoint_param.temporary_deceleration = ;
						curr_waypoint.waypoint_param.temporary_fixed_velocity = 0;
						double bunsi = std::pow(current_velocity_ave_, 2) - std::pow(cmdacc.vehicle_ahead_velocity, 2);
						double bunbo = 2 * stop_line_info.distance_;
						double dec_acc = bunsi / bunbo;
						std::cout << "v2," << cmdacc.vehicle_ahead_velocity << std::endl;
						std::cout << "v02," << current_velocity_ave_ << std::endl;
						std::cout << "dis," << stop_line_info.distance_ << std::endl;
						std::cout << "dec_acc," << dec_acc << std::endl;
						if(dec_acc < 0) dec_acc = 0;
						curr_waypoint.waypoint_param.temporary_deceleration = 0.30;//dec_acc;
					}

					std::stringstream line_th_status_ss;
					line_th_status_ss << distance_th_str << "," << distance_th << "  " << cmd_pass_vel_str << "," << cmd_pass_vel;
					std_msgs::String line_th_status_msg;
					line_th_status_msg.data = line_th_status_ss.str();
					pub_line_th_status_.publish(line_th_status_msg);

					break;
				}
				else
				{
					std::cout << "vel_ave," << waypoints_velocity_mps*3.6 << std::endl;
					curr_waypoint.twist.twist.linear.x = waypoints_velocity_mps;
				}
			}
		}

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
		if(foreground_obstract_list_.getSize() == 0)
		{
			accel_stroke_cap.data = MAX_CAN_ACCEL_STROKE_CAP;
		}
		else
		{
			autoware_msgs::TransformMobileyeObstacle obs = foreground_obstract_list_.getLatestObs();
			if(obs.velocity_mps <= CAN_SEND_TH_KMH / 3.6
				&& obs.orig_data.object_accel_x <= CAN_SEND_TH_ACC
				&& stop_line_info.distance_ <= CAN_SEND_TH_DISTANCE)
			{
				accel_stroke_cap.data = MIN_CAN_ACCEL_STROKE_CAP;
			}
			else
				accel_stroke_cap.data = MAX_CAN_ACCEL_STROKE_CAP;
		}
		pub_can_stroke_cap_.publish(accel_stroke_cap);
	}

	void callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
	{
		//std::cout << "callback way" << std::endl;
		StopLineInfo stop_line_info = searchCarFollowing(msg);
		/*if(foreground_obstract_list_.getSize() > 0)//コマンド計算結果表示
		{
			const autoware_msgs::MobileyeCmdParam cmdacc = mathTargetAcc(foreground_obstract_list_);
			cmdPubnlish(cmdacc);
		}*/
		createCarTrackingWaypoints(msg, stop_line_info);
		publishAccelCanCap(stop_line_info);
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
			current_acc_ave_ = (vel - current_velocity_ave_) / time_diff;
		}
		else
		{
			current_acc_ave_ = 0;
		}
		current_velocity_ave_ = vel;
		//current_velocity_ = *msg;
	}

	//mobileye車両オブジェクトを追加
	void addObstract(const autoware_msgs::TransformMobileyeObstacle::ConstPtr &msg)
	{
		bool add_flag = false;
		for(size_t info_cou=0; info_cou<obstract_info_list_.size(); info_cou++)
		{
			ObstractInfo &obs_list = obstract_info_list_[info_cou];
			const autoware_msgs::TransformMobileyeObstacle &obs = obs_list.getObs(0);
			if(obs.orig_data.obstacle_id == msg->orig_data.obstacle_id)//オブジェクトIDが同一なら追加
			{
				/*obs_list.push_back(*msg);
				if(obs_list.size() > FORREGROUND_VELOCITY_LIST_SIZE)
					obs_list.erase(obs_list.begin());*/
				obs_list.push(*msg);
				add_flag = true;
			}
		}

		if(add_flag == false)//オブジェクトIDが既存になければ追加
		{
			/*std::vector<autoware_msgs::TransformMobileyeObstacle> new_list;
			new_list.push_back(*msg);
			obstract_info_.push_back(new_list);*/
			ObstractInfo new_info;
			new_info.push(*msg);
			obstract_info_list_.push_back(new_info);
		}
	}

	//最近傍の正面車両を探索
	StopLineInfo searchCarFollowing(const autoware_msgs::Lane::ConstPtr &lane)
	{
		int hit_obs_index = -1; //判定車両ID
		int hit_waypoint_index = -1; //判定waypoint
		double x_adj = 0; //判定waypointのx微調整位置

		for(int waycou=1; waycou<lane->waypoints.size(); waycou++)
		{
			const autoware_msgs::Waypoint waypoint = lane->waypoints[waycou];
			const autoware_msgs::Waypoint prev_waypoint = lane->waypoints[waycou-1];
			const geometry_msgs::Point way_po = waypoint.pose.pose.position;
			const geometry_msgs::Point prev_way_po = prev_waypoint.pose.pose.position;

			for(int obscou=0; obscou<obstract_info_list_.size(); obscou++)
			{
				const ObstractInfo &obs_info = obstract_info_list_[obscou];
				//geometry_msgs::Quaternion curr_qua = waypoint.pose.pose.orientation;
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

				if(std::abs(obs_diff_po_rot.y()) <= config_.judg_obj_pose_y)
				{
					if(obs_diff_po_rot.x() >= 0)
					{
						if(obs_diff_po_rot.x() <= way_diff_po_rot.x())
						{
							hit_obs_index = obscou;
							hit_waypoint_index = waycou;
							x_adj = way_diff_po_rot.x() - obs_diff_po_rot.x();
						}
					}
				}
			}

			if(hit_obs_index != -1) break;
		}

		std::cout << "hit," << hit_obs_index << std::endl;
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

			foreground_obstract_list_ = obstract_info_list_[hit_obs_index];
			StopLineInfo info;
			info.waypoint_index_ = stop_index;
			info.x_adj_ = dis_adj;
			info.distance_ = dis_sum;
			pub_front_mobileye_obstacle_.publish(foreground_obstract_list_.getLatestObs());
			return info;
		}
		else
		{
			foreground_obstract_list_.clear();
			StopLineInfo info;
			info.waypoint_index_ = -1;
			info.x_adj_ = -1;
			info.distance_ = -1;
			autoware_msgs::TransformMobileyeObstacle msg;
			msg.orig_data.obstacle_id = USHRT_MAX;
			pub_front_mobileye_obstacle_.publish(msg);
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
		addObstract(msg);
	}

	//正面判定された車両情報がconfig_.obs_delete_time秒間更新が無かった場合、その車両情報を消去する
	void callbackObsDeleteTimer(const ros::TimerEvent &e)
	{
		ros::Time ros_nowtime = ros::Time::now();//e.current_real;
		for(long i=obstract_info_list_.size()-1; i>=0; i--)
		{
			ObstractInfo obs_info = obstract_info_list_[i];
			autoware_msgs::TransformMobileyeObstacle obs = obs_info.getLatestObs();//obs_info.getObs(obs_info.getSize()-1);
			ros::Duration ros_time_diff = ros_nowtime - obs.header.stamp;
			double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
			if(time_diff > config_.obs_delete_time) obstract_info_list_.erase(obstract_info_list_.begin() + i);
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
		, current_velocity_ave_(0)
		, current_acc_ave_(0)
		, velocity0_timer_(ros::Time(0))
		, target_deceleration_(0.1)
	{
		front_bumper_to_baselink_ = nh_.param<double>("/vehicle_info/front_bumper_to_baselink", 4.55);

		sub_config_ = nh_.subscribe("/config/mobileye_tracker", 10, &CarTrackinig::callbackConfig, this);
		sub_waypoints_ = nh_.subscribe("/safety_waypoints", 10, &CarTrackinig::callbackWaypoints, this);
		sub_current_velocity_ = nh_.subscribe("/current_velocity", 10, &CarTrackinig::callbackCurrentVelocity, this);
		sub_mobileye_obstacle_ = nh_.subscribe("/transform_mobileye_obstacle", 10, &CarTrackinig::callbackMobileyeObstacle, this);
		sub_target_deceleration_ = nh_.subscribe("/car_target_deceleration", 10, &CarTrackinig::callbackTargetDeceleration, this);

		pub_waypoints_ = nh_.advertise<autoware_msgs::Lane>("/car_tracking_waypoints", 10);
		pub_mobileye_cmd_param_ = nh_.advertise<autoware_msgs::MobileyeCmdParam>("/mobileye_tracker/cmd_param", 10);
		pub_waypoint_vel_ = nh_.advertise<std_msgs::String>("/mobileye_tracker/waypoint_vel", 10);
		pub_track_pattern_ = nh_.advertise<std_msgs::String>("/mobileye_tracker/track_pattern", 10);
		pub_front_mobileye_obstacle_ = nh_.advertise<autoware_msgs::TransformMobileyeObstacle>("/mobileye_tracker/front_car", 10);
		pub_line_th_status_ = nh_.advertise<std_msgs::String>("/mobileye_tracker/line_th_status", 10);
		pub_can_stroke_cap_ = nh_.advertise<std_msgs::Int16>("/mobileye_tracker/accel_stroke_cap", 10);
		pub_car_cruise_status_ = nh_.advertise<autoware_msgs::CarCruiseStatus>("/car_cruise_status", 10);

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