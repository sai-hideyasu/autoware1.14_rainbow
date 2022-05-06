#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/TrafficLight.h>
#include <tf/transform_broadcaster.h>
#include <autoware_msgs/StopperDistance.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/CANInfo.h>
#include <limits.h>

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

class StopperDistance
{
public:
	const int TRAFFIC_LIGHT_RED = 0;
	const int TRAFFIC_LIGHT_GREEN = 1;
	const int TRAFFIC_LIGHT_YELLOW = 10;
	const int TRAFFIC_LIGHT_YELLOW_RED = 11;
	const int TRAFFIC_LIGHT_UNKNOWN = 2;
private:
	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_waypoint_, sub_light_color_, sub_camera_light_color_, sub_temporari_flag_;
	ros::Subscriber sub_obstacle_waypoint_, sub_mobileye_velocity_, sub_temporary_fixed_velocity_;
	ros::Subscriber sub_can502_, sub_can_info_, sub_oncoming_stop_, sub_current_velocity_;

	ros::Publisher pub_stopline_distance_, pub_obstacle_offset_;

	double front_bumper_to_baselink_ = 6.18;//6.25 - 1.7;

	bool oncoming_stop_;
	void callbackOncomingStop(const std_msgs::Bool &msg)
	{
		oncoming_stop_ = msg.data;
	}

	double temporary_fixed_velocity_;
	void callbackTemporaryFixedVelocity(const std_msgs::Float64& msg)
	{
		temporary_fixed_velocity_ = msg.data;
	}

	double obstacle_velocity_;
	void callbackMobileyeVelocity(const std_msgs::Float64& msg)
	{
		obstacle_velocity_ = msg.data;
	}

	int light_color_;
	void callbackCameraLightColor(const autoware_msgs::TrafficLight& msg)
	{
		light_color_ = msg.traffic_light;
		std::cout << light_color_ << std::endl;
	}
	void callbackLightColor(const autoware_msgs::TrafficLight& msg)
	{
		light_color_ = msg.traffic_light;
		std::cout << light_color_ << std::endl;
	}

	int temporary_flag_;
	void callbackTemporaryFlag(const std_msgs::Int32& msg)
	{
		temporary_flag_ = msg.data;
	}

	int obstacle_waypoint_;
	//const int obstacle_offset_ = 2;
	void callbackObstacleWaypoint(const std_msgs::Int32& msg)
	{
		obstacle_waypoint_ = msg.data;
	}

	void waypointCallback(const autoware_msgs::Lane& msg)
	{
		autoware_msgs::StopperDistance pubmsg;
		pubmsg.distance = -1;
		pubmsg.send_process = autoware_msgs::StopperDistance::UNKNOWN;
		pubmsg.fixed_velocity = 0;

		double dis = 0;
		for(int i=1; i<msg.waypoints.size(); i++)
		{
			tf::Vector3 v1(msg.waypoints[i-1].pose.pose.position.x,
						   msg.waypoints[i-1].pose.pose.position.y, 0);

			tf::Vector3 v2(msg.waypoints[i].pose.pose.position.x,
						   msg.waypoints[i].pose.pose.position.y, 0);

			dis += tf::tfDistance(v1, v2);

			autoware_msgs::WaypointParam waypoint_param = msg.waypoints[i].waypoint_param;
			if(waypoint_param.signal_stop_line > 0 && (light_color_ == TRAFFIC_LIGHT_RED || light_color_ == TRAFFIC_LIGHT_YELLOW_RED))
			{
				pubmsg.distance = dis - front_bumper_to_baselink_ + waypoint_param.stop_line_adjustment;
				pubmsg.send_process = autoware_msgs::StopperDistance::SIGNAL;
				pubmsg.fixed_velocity = 0;
				if(pubmsg.distance < 0) pubmsg.distance = 0;
				double v = current_vel_ave_ - pubmsg.fixed_velocity;
				double sqrt_val = current_vel_ave_ * current_vel_ave_ + 2 * current_acc_ave_ * pubmsg.distance;
				if(std::abs(current_acc_ave_) <= 1.0E-4 || sqrt_val < 0) pubmsg.reach_time = pubmsg.distance / current_vel_ave_;
				else pubmsg.reach_time = (-current_vel_ave_ + std::sqrt(sqrt_val)) / current_acc_ave_;

				double curr_space, front_space;
				pubmsg.distance_rss = targetFreeSpaceRSS(current_vel_ave_, 0, -0.325, 0, 1.0, 1.0, 0, curr_space, front_space);
				break;
			}
			else if(waypoint_param.object_stop_line > 0)
			{
				pubmsg.distance = dis - front_bumper_to_baselink_ + waypoint_param.stop_line_adjustment;
				pubmsg.send_process = autoware_msgs::StopperDistance::OBSTACLE;
				pubmsg.fixed_velocity = temporary_fixed_velocity_;
				if(pubmsg.distance < 0) pubmsg.distance = 0;
				double v = current_vel_ave_ - pubmsg.fixed_velocity;
				double sqrt_val = current_vel_ave_ * current_vel_ave_ + 2 * current_acc_ave_ * pubmsg.distance;
				if(std::abs(current_acc_ave_) <= 1.0E-4 || sqrt_val < 0) pubmsg.reach_time = pubmsg.distance / current_vel_ave_;
				else pubmsg.reach_time = (-current_vel_ave_ + std::sqrt(sqrt_val)) / current_acc_ave_;

				double curr_space, front_space;
				pubmsg.distance_rss = targetFreeSpaceRSS(current_vel_ave_, 0, -0.325, 0, 1.0, 1.0, 0, curr_space, front_space);
				break;
			}
			else if(waypoint_param.temporary_stop_line > 0 && temporary_flag_ == 1)
			{
				pubmsg.distance = dis - front_bumper_to_baselink_ + waypoint_param.stop_line_adjustment;
				pubmsg.send_process = autoware_msgs::StopperDistance::TEMPORARY_STOPPER;
				pubmsg.fixed_velocity = temporary_fixed_velocity_;
				if(pubmsg.distance < 0) pubmsg.distance = 0;
				double v = current_vel_ave_ - pubmsg.fixed_velocity;
				double sqrt_val = current_vel_ave_ * current_vel_ave_ + 2 * current_acc_ave_ * pubmsg.distance;
				if(std::abs(current_acc_ave_) <= 1.0E-4 || sqrt_val < 0) pubmsg.reach_time = pubmsg.distance / current_vel_ave_;
				else pubmsg.reach_time = (-current_vel_ave_ + std::sqrt(sqrt_val)) / current_acc_ave_;

				double curr_space, front_space;
				pubmsg.distance_rss = targetFreeSpaceRSS(current_vel_ave_, temporary_fixed_velocity_, -0.325, 0, 1.0, 1.0, 0, curr_space, front_space);
				break;
			}
			else if(waypoint_param.oncoming_stop_line > 0 && oncoming_stop_ == true)
			{
				pubmsg.distance = dis - front_bumper_to_baselink_ + waypoint_param.stop_line_adjustment;
				pubmsg.send_process = autoware_msgs::StopperDistance::ONCOMING;
				pubmsg.fixed_velocity = temporary_fixed_velocity_;
				if(pubmsg.distance < 0) pubmsg.distance = 0;
				double v = current_vel_ave_ - pubmsg.fixed_velocity;
				double sqrt_val = current_vel_ave_ * current_vel_ave_ + 2 * current_acc_ave_ * pubmsg.distance;
				if(std::abs(current_acc_ave_) <= 1.0E-4 || sqrt_val < 0) pubmsg.reach_time = pubmsg.distance / current_vel_ave_;
				else pubmsg.reach_time = (-current_vel_ave_ + std::sqrt(sqrt_val)) / current_acc_ave_;

				double curr_space, front_space;
				pubmsg.distance_rss = targetFreeSpaceRSS(current_vel_ave_, 0, -0.325, 0, 1.0, 1.0, 0, curr_space, front_space);
				break;
			}
			else if(obstacle_waypoint_ > 0 && obstacle_waypoint_ == i)
			{
				//pubmsg.distance = dis - (front_bumper_to_baselink_ + obstacle_offset_) + waypoint_param.stop_line_adjustment;
				double km_h;
				switch(can_read_flag)
				{
				case StopperDistance::CANREAD_LIESSE:
					km_h = can502_.velocity_mps/3.6; break;
				case StopperDistance::CANREAD_PRIUS:
					km_h = can_info_.speed;
				}
				double a = (km_h/2.0 > 3.0) ? km_h/2.0 : 3.0;
				//double obstacle_offset = (a > 4) ? 4 : a;
				double obstacle_offset = 4;
				pubmsg.distance = dis - (front_bumper_to_baselink_ + obstacle_offset);
				pubmsg.send_process = autoware_msgs::StopperDistance::OBSTACLE;
				pubmsg.fixed_velocity = obstacle_velocity_;
				if(pubmsg.distance < 0) pubmsg.distance = 0;
				double v = current_vel_ave_ - pubmsg.fixed_velocity;
				double sqrt_val = current_vel_ave_ * current_vel_ave_ + 2 * current_acc_ave_ * pubmsg.distance;
				if(std::abs(current_acc_ave_) <= 1.0E-4 || sqrt_val < 0) pubmsg.reach_time = pubmsg.distance / current_vel_ave_;
				else pubmsg.reach_time = (-current_vel_ave_ + std::sqrt(sqrt_val)) / current_acc_ave_;

				double curr_space, front_space;
				pubmsg.distance_rss = targetFreeSpaceRSS(current_vel_ave_, 0, -0.325, 0, 1.0, 1.0, 0, curr_space, front_space);
				break;
			}
		}
		pub_stopline_distance_.publish(pubmsg);
	}

	static const int CANREAD_NOT =0;
	static const int CANREAD_LIESSE =1;
	static const int CANREAD_PRIUS =2;
	autoware_can_msgs::MicroBusCan502 can502_;
	int can_read_flag;
	void callbackCan502(const autoware_can_msgs::MicroBusCan502 &msg)
	{
		can502_ = msg;
		can_read_flag = StopperDistance::CANREAD_LIESSE;
	}

	autoware_can_msgs::CANInfo can_info_;
	void callbackCanInfo(const autoware_can_msgs::CANInfo &msg)
	{
		can_info_ = msg;
		can_read_flag = StopperDistance::CANREAD_PRIUS;
	}

	const static size_t CURRENT_VELOCITY_LIST_SIZE = 10;
	std::vector<double> current_velocity_list;
	double current_vel_ave_;
	double current_acc_ave_;
	void callbackCurrentVelocity(const geometry_msgs::TwistStamped &msg)
	{
		current_velocity_list.push_back(msg.twist.linear.x);
		if(current_velocity_list.size() > CURRENT_VELOCITY_LIST_SIZE)
			current_velocity_list.erase(current_velocity_list.begin());

		current_acc_ave_ = 0;
		current_vel_ave_ = 0;
		if(current_velocity_list.size() >= 2)
		{
			for(int i=0; i<current_velocity_list.size(); i++)
			{
				double curr_v = current_velocity_list[i];
				current_vel_ave_ = curr_v;
				if(i != 0)
				{
					double prev_v = current_velocity_list[i-1];
					current_acc_ave_ += curr_v - prev_v;
				}
			}
			current_vel_ave_ /= current_velocity_list.size();
			current_acc_ave_ /= current_velocity_list.size()-1;
		}
	}
public:
	StopperDistance(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: obstacle_velocity_(0)
		, temporary_fixed_velocity_(0)
		, light_color_(1)
		, temporary_flag_(0)
		, obstacle_waypoint_(-1)
		, can_read_flag(CANREAD_NOT)
		, current_acc_ave_(0)
	{
		nh_ = nh;  private_nh_ = p_nh;

		private_nh_.param<double>("/vehicle_info/front_bumper_to_baselink", front_bumper_to_baselink_, 6.18);
		std::cout << "front," << front_bumper_to_baselink_ << std::endl;
		pub_stopline_distance_ = nh_.advertise<autoware_msgs::StopperDistance>("/stopper_distance", 1);
		pub_obstacle_offset_ = nh_.advertise<std_msgs::Float64>("/obstacle_offset", 1);

		sub_waypoint_ = nh_.subscribe("/final_waypoints", 1, &StopperDistance::waypointCallback, this);
		sub_light_color_ = nh_.subscribe("/light_color", 1, &StopperDistance::callbackLightColor, this);
		//sub_camera_light_color_ = nh_.subscribe("/camera_light_color", 1, &StopperDistance::callbackCameraLightColor, this);
		sub_temporari_flag_ = nh_.subscribe("/temporary_flag", 1, &StopperDistance::callbackTemporaryFlag, this);
		sub_obstacle_waypoint_ = nh_.subscribe("/obstacle_waypoint", 1, &StopperDistance::callbackObstacleWaypoint, this);
		sub_mobileye_velocity_ = nh_.subscribe("/obstacle_velocity", 1, &StopperDistance::callbackMobileyeVelocity, this);
		sub_temporary_fixed_velocity_ = nh_.subscribe("/temporary_fixed_velocity", 1, &StopperDistance::callbackTemporaryFixedVelocity, this);
		sub_can502_ = nh_.subscribe("/microbus/can_receive502", 1, &StopperDistance::callbackCan502, this);
		sub_can_info_ = nh_.subscribe("/can_info", 1, &StopperDistance::callbackCanInfo, this);
		sub_oncoming_stop_ = nh_.subscribe("/oncoming_stop", 1, &StopperDistance::callbackOncomingStop, this);
		sub_current_velocity_ = nh_.subscribe("/current_velocity", 1, &StopperDistance::callbackCurrentVelocity, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stopper_distance");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	StopperDistance sd(nh, private_nh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
