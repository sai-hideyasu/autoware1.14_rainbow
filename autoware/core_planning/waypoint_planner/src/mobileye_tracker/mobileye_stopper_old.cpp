#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/TransformMobileyeObstacle.h>
#include <autoware_msgs/Lane.h>
#include <autoware_config_msgs/ConfigMobileyeStopper.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct StopInfo
{
	int way_id_;
	uint16_t obs_id_;
	geometry_msgs::Point pos_;
	double distance_;
	double vel_;
};

void geometryQuatToRpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

geometry_msgs::Point pointRol(geometry_msgs::Point p, double yaw)
{
	Eigen::Vector3d po(p.x, p.y, p.z);
	Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_rot = qua * po;
	geometry_msgs::Point ret;
	ret.x = po_rot.x();
	ret.y = po_rot.y();
	ret.z = po_rot.z();
}

geometry_msgs::Pose waypointPoseCorrect(const geometry_msgs::Pose p, const double corr)
{
	Eigen::Vector3d po(p.position.x, p.position.y, p.position.z);
	double roll, pitch, yaw;
	geometryQuatToRpy(roll, pitch, yaw, p.orientation);
	Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_rot = qua * po;
	po_rot += Eigen::Vector3d(corr, 0, 0);
	Eigen::Quaterniond qua_rev = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_move = qua_rev * po_rot;
	geometry_msgs::Pose ret;
	ret.position.x = po_move.x();
	ret.position.y = po_move.y();
	ret.position.z = po_move.z();
	ret.orientation = p.orientation;
	return ret;
}

double mathDistanceXYZ(const geometry_msgs::Point pose1, const geometry_msgs::Point pose2)
{
	double x = pose1.x - pose2.x;
	double y = pose1.y - pose2.y;
	double z = pose1.z - pose2.z;
	return sqrt(x*x + y*y + z*z);
}

double mathDistanceXY(const geometry_msgs::Point pose1, const geometry_msgs::Point pose2)
{
	double x = pose1.x - pose2.x;
	double y = pose1.y - pose2.y;
	//double z = pose1.position.z - pose2.position.z;
	return sqrt(x*x + y*y);
}

class MobileyeStopper
{
private:
	static const double COEFFICIENT_FRICTION_SUNNY = 0.7;
	static const double COEFFICIENT_FRICTION_RAIN = 0.5;
	static const double COEFFICIENT_FRICTION_CAREFUL = 0.4;

	const static int AVE_VEL_MAX_ = 5;
	const static double CAR_COUNTER_MAGN_ = 3.0;//1;
	const static double RELEASE_TIME_ = 1.0;

	ros::NodeHandle nh_, pnh_;

	ros::Subscriber sub_transform_mobileye_obstacle_, sub_current_pose_, sub_current_velocity_, sub_waypoints_;
	ros::Subscriber sub_config_;
	ros::Timer timer;

	double front_length_;

	autoware_config_msgs::ConfigMobileyeStopper config_;
	void callbackConfig(const autoware_config_msgs::ConfigMobileyeStopper::ConstPtr &msg)
	{
		config_ = *msg;
	}

	void resetStopInfo(StopInfo &stop_info)
	{
		stop_info.way_id_ -1;
		stop_info.pos_.x = stop_info.pos_.y = stop_info.pos_.z = 0;
		stop_info.distance_ = -1;
		stop_info.vel_ = 0;
	}

	//障害物探索
	/*void obstacleSearch(const autoware_msgs::TransformMobileyeObstacleList &obs_data_, StopInfo &ret)
	{
		resetStopInfo(ret);

		std::vector<double> dt_list(obs_data_.data.size(), DBL_MAX);
		std::vector<int> id_list(obs_data_.data.size(), -1);

		for(int i=0; i<waypoints_.waypoints.size(); i++)
		{
			const geometry_msgs::Pose &way_pose = waypoints_.waypoints[i].pose.pose;
			const int way_id = waypoints_.waypoints[i].waypoint_param.id;

			//for(geometry_msgs::Pose obs_pose : obs_pose_list)
			for(int obs_ind=0; obs_ind<obs_data_.data.size(); obs_ind++)
			{
				const geometry_msgs::Pose &obs_pose = obs_data_.data[obs_ind].pose;
				double dt = mathDistanceXY(obs_pose.position, way_pose.position);
				if(dt <= config_.search_distance)
				{
					dt_list[obs_ind] = dt;
					id_list[obs_ind] = way_id;
				}
			}
		}

		for(int i=0; i<waypoints_.waypoints.size(); i++)
		{
			const int way_id = waypoints_.waypoints[i].waypoint_param.id;
			for(int obs_ind=0; obs_ind<obs_data_.data.size(); obs_ind++)
			{
				if(way_id == id_list[obs_ind])
				{
					ret.way_id_ = id_list[obs_ind];
					ret.distance_ = obs_data_.data[obs_ind].orig_data.obstacle_pos_x;//dt_list[obs_ind];
					ret.pos_ = obs_data_.data[obs_ind].pose.position;
					ret.vel_ = obs_data_.data[obs_ind].orig_data.obstacle_rel_vel_x;
					ret.obs_id_ = obs_data_.data[obs_ind].orig_data.obstacle_id;
					return;
				}
			}
		}
	}*/

	/*void obstacleSearch(const autoware_msgs::TransformMobileyeObstacle &obs_data_, StopInfo &ret)
	{
		double min_dt = DBL_MAX;
		for(autoware_msgs::TransformMobileyeObstacle tmo : obs_data_)
		{
			double rel_vel_x = tmo.orig_data.obstacle_rel_vel_x * 3.6;
			if(current_velocity_.twist.linear.x < -rel_vel_x - CAR_COUNTER_MAGN_) continue;

			double dt = mathDistanceXY(tmo.pose.position, current_pose_.pose.position);
			if(dt < min_dt)
			{
				min_dt = dt;
				ret.way_id_ = -1;
				ret.distance_ = tmo.orig_data.obstacle_pos_x;//obs_data_.data[obs_ind].orig_data.obstacle_pos_x;//dt_list[obs_ind];
				ret.pos_ = tmo.pose.position;//obs_data_.data[obs_ind].pose.position;
				ret.vel_ = tmo.orig_data.obstacle_rel_vel_x * 3.6;//obs_data_.data[obs_ind].orig_data.obstacle_rel_vel_x;
				ret.obs_id_ = tmo.orig_data.obstacle_id;//obs_data_.data[obs_ind].orig_data.obstacle_id;
			}
		}
	}*/

	double mathAcc(std::vector<double> &list, std::vector<double> &dt_list, double vel, double dt)
	{
		list.emplace_back(vel);
		dt_list.emplace_back(dt);
		if(list.size() > AVE_VEL_MAX_)
		{
			list.erase(list.begin());
			dt_list.erase(dt_list.begin());
		}
		//for(int i=0; i<dt_list.size(); i++) std::cout << i << ","<< dt_list[i] << std::endl;
		//std::cout << "val," << vel << "," << dt << std::endl;
		//std::cout << "dt_list_size,"<< list.size() << "," << dt_list.size() << std::endl;
		double sum = 0;
		for(int i=0; i<list.size(); i++)
		{
			sum += list[i] / dt_list[i];
			//std::cout << "list," << list[i] << " dt_list," << dt_list[i] << std::endl;
		}
		//std::cout << std::endl;
		//std::cout << "sum," << sum << std::endl;
		//return sum / list.size();
	}

	autoware_msgs::TransformMobileyeObstacle prev_obstacle_;
	std::vector<double> obstacle_vel_list_;
	std::vector<double> obstacle_dt_list_;
	ros::Time obstacle_time_;
	unsigned int step_counter_;
	void callbackMobileyeObstacle(const autoware_msgs::TransformMobileyeObstacle::ConstPtr &msg)
	{
		//std::cout << "step," << step_counter_ << std::endl;
		step_counter_++;
		ros::Time nowtime = ros::Time::now();

		bool counter_flag = false;
		double rel_vel_x = msg->orig_data.obstacle_rel_vel_x;
		double prev_rel_vel_x = prev_obstacle_.orig_data.obstacle_rel_vel_x;
		//if(current_velocity_.twist.linear.x < -rel_vel_x - CAR_COUNTER_MAGN_) counter_flag = true;
		if(current_velocity_.twist.linear.x + rel_vel_x < -CAR_COUNTER_MAGN_) counter_flag = true;
		/*if(msg->orig_data.obstacle_id == 27)
		{
			std::cout << "lineax," << current_velocity_.twist.linear.x << ".relx," << rel_vel_x << std::endl;
			std::cout << "status," << (int)msg->orig_data.obstacle_status << std::endl;
			std::cout << "counter_diff,"<< current_velocity_.twist.linear.x + rel_vel_x << std::endl;
		}*/

		bool id_flag = false;
		if(msg->orig_data.obstacle_id == prev_obstacle_.orig_data.obstacle_id) id_flag = true;

		bool distance_flag = false;
		double distance1 = mathDistanceXY(msg->pose.position, current_pose_.pose.position);
		double distance2 = mathDistanceXY(prev_obstacle_.pose.position, current_pose_.pose.position);
		if(distance1 < distance2) distance_flag = true;
		bool first_flag = false;
		if(prev_obstacle_.orig_data.obstacle_id == UINT16_MAX) first_flag = true;

		std::cout << "counter," << (int)counter_flag << ",id," << id_flag << ",dis," << distance_flag << ",first," << first_flag << std::endl;
		std::cout << "obs_id," << msg->orig_data.obstacle_id << std::endl;
		if(!counter_flag)
		{
			if(first_flag)
			{
				prev_obstacle_ = *msg;
				obstacle_time_ = nowtime;
			}
			//if(id_flag || distance_flag)
			else if(id_flag)
			{
				std::cout << "pro_id," << msg->orig_data.obstacle_id << std::endl;

				ros::Duration ros_obs_dt = msg->header.stamp - prev_obstacle_.header.stamp;
				double obs_dt = ros_obs_dt.sec + ros_obs_dt.nsec * 1E-9;
				//std::cout << "obs_dt," << obs_dt << std::endl;
				double obstacle_acc = mathAcc(obstacle_vel_list_, obstacle_dt_list_, rel_vel_x - prev_rel_vel_x, obs_dt);
				//std::cout << "obs_acc," << obstacle_acc << std::endl;

				//L0
				double L0 = mathDistanceXY(msg->pose.position, current_pose_.pose.position);
				//std::cout << "L0," << L0 << std::endl;

				//Lt
				const double REACTION_TIME = 0.2;
				const double STOP_COEFFICIENT = 3.6*3.6*9.8*2;
				double vel_mps = current_velocity_.twist.linear.x / 3.6;	//!< 車速の秒測値
				double free_run_distance = vel_mps * REACTION_TIME;
				double braking_distance = pow(current_velocity_.twist.linear.x, 2) / (COEFFICIENT_FRICTION_SUNNY * STOP_COEFFICIENT);
				double Lt = free_run_distance + braking_distance;
				//std::cout << "Lt," << Lt << std::endl;

				//自車の目標加速度
				//double target_acc = obstacle_acc + rel_vel_x * rel_vel_x / (Lt - L0) / 2.0;	//!< 目標加速度
				//std::cout << "obs_acc," << obstacle_acc << std::endl; 
				//std::cout << "target acc," << target_acc << std::endl;

				
				prev_obstacle_ = *msg;
				obstacle_time_ = nowtime;
			}
			else if(distance_flag)
			{
				prev_obstacle_ = *msg;
				obstacle_time_ = nowtime;
			}
		}

		//if(waypoints_.waypoints.size() < 2) return;

		/*StopInfo stop_info;
		obstacleSearch(*msg, stop_info);
		//std::cout << "way_id," << stop_info.way_id_ << std::endl;
		std::cout << "pose," << stop_info.pos_.x << "," << stop_info.pos_.y << "," << stop_info.pos_.z << std::endl;
		std::cout << "dis," << stop_info.distance_ << std::endl;
		std::cout << "vel," << stop_info.vel_ << std::endl;
		std::cout << "obs_id," << stop_info.obs_id_ << std::endl;
		std::cout << -current_velocity_.twist.linear.x << "," << stop_info.vel_ << std::endl;
		//if(stop_info.way_id_ == waypoints_.waypoints.size()-1) return;

		//if(stop_info.way_id_ != -1)
		{
			//transformの時間は現在nowだからあとで戻せ！！！！！！
			//mobileye obstracle acc
			ros::Duration ros_obs_dt = msg->header.stamp - prev_obstacle_time_;
			double obs_dt = ros_obs_dt.sec + ros_obs_dt.nsec * 1E-9;
			std::cout << "obs_dt," << obs_dt << std::endl;
			double obstacle_acc = mathAcc(obstacle_vel_list_, obstacle_dt_list_, stop_info.vel_ - prev_stop_info_.vel_, obs_dt);

			//if(prev_stop_info_.way_id_ != -1 && prev_stop_info_.obs_id_ == stop_info.obs_id_)
			if(prev_stop_info_.obs_id_ == stop_info.obs_id_)
			{
				//L0
				double L0 = mathDistanceXY(stop_info.pos_, current_pose_.pose.position);
				//std::cout << stop_info.pos_.x << "," << current_pose_.pose.position.x << std::endl;
				std::cout << "L0," << L0 << std::endl;

				//Lt
				const double REACTION_TIME = 0.2;
				const double STOP_COEFFICIENT = 3.6*3.6*9.8*2;
				double vel_mps = current_velocity_.twist.linear.x / 3.6;	//!< 車速の秒測値
				double free_run_distance = vel_mps * REACTION_TIME;
				double braking_distance = pow(current_velocity_.twist.linear.x, 2) / (COEFFICIENT_FRICTION_SUNNY * STOP_COEFFICIENT);
				//double inter_vehicle_distance = free_run_distance + braking_distance;
				double Lt = free_run_distance + braking_distance;
				std::cout << "Lt," << Lt << std::endl;

				//自車の目標加速度
				double target_acc = obstacle_acc + stop_info.vel_/(Lt - L0) / 2.0;	//!< 目標加速度
				std::cout << "obs_acc," << obstacle_acc << std::endl; 
				std::cout << "target acc," << target_acc << std::endl;
			}
			else obstacle_vel_list_.clear();
			
		}*/
		/*else
		{
			obstacle_vel_list_.clear();
		}*/
	}

	geometry_msgs::PoseStamped current_pose_;
	void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		current_pose_ = *msg;
	}

	geometry_msgs::TwistStamped current_velocity_;
	/*std::vector<double> current_vel_list_;
	std::vector<double> current_dt_list_;
	double current_acc_;*/
	void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		/*ros::Duration rostime = msg->header.stamp - current_velocity_.header.stamp;
		double time = rostime.sec + rostime.nsec * 1E-9;
		current_acc_ = mathAcc(current_vel_list_, current_dt_list_, msg->twist.linear.x - current_velocity_.twist.linear.x, time);
		//std::cout << "current_acc," << current_acc_ << std::endl; */
		current_velocity_ = *msg;

		/*current_vel_list_.emplace_back(msg->twist.linear.x);
		if(current_vel_list_.size() > AVE_VEL_MAX_) current_vel_list_.erase(current_vel_list_.end());
		double sum = 0;
		for(double val : current_vel_list_) sum += val;
		opponent_acc_ = sum / time;
		current_velocity_ = *msg;*/
	}

	autoware_msgs::Lane waypoints_;
	void callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
	{
		waypoints_ = *msg;
	}

	void callbackTimer(const ros::TimerEvent& e)
	{
		ros::Duration dt = ros::Time::now() - obstacle_time_;
		//std::cout << "dt," << dt << "," << ros::Duration(RELEASE_TIME_) << std::endl;
		if(dt > ros::Duration(RELEASE_TIME_))
		{
			prev_obstacle_.orig_data.obstacle_id = UINT16_MAX;
			obstacle_vel_list_.clear();
			obstacle_dt_list_.clear();
		}
	}
public:
	MobileyeStopper(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, front_length_(3.935)
		, step_counter_(0)
	{
		nh_.param<double>("/vehicle_info/front_bumper_to_baselink", front_length_, 3.935);
		prev_obstacle_.orig_data.obstacle_id = UINT16_MAX;

		sub_config_ = nh_.subscribe("/config/mobileye_stopper", 1, &MobileyeStopper::callbackConfig, this);
		sub_transform_mobileye_obstacle_ = nh_.subscribe("/transform_mobileye_obstacle", 1, &MobileyeStopper::callbackMobileyeObstacle, this);
		sub_current_pose_ = nh_.subscribe("/current_pose", 1, &MobileyeStopper::callbackCurrentPose, this);
		sub_current_velocity_ = nh_.subscribe("/current_velocity", 1, &MobileyeStopper::callbackCurrentVelocity, this);
		sub_waypoints_ = nh_.subscribe("/temporary_stop_waypoints", 1, &MobileyeStopper::callbackWaypoints, this);
		timer = nh_.createTimer(ros::Duration(0.05), &MobileyeStopper::callbackTimer, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mobileye_stopper");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	MobileyeStopper stopper(nh, pnh);
	ros::spin();
	return 0;
}