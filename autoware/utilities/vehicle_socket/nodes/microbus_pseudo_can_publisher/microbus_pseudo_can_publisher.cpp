#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <autoware_config_msgs/ConfigMicrobusPseudoCanPublisher.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_can_msgs/MicroBusPseudoParams.h>
#include <autoware_can_msgs/MicroBusPseudoOperation.h>
#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/Gear.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "microbus_params.h"

class MicrobusPseudoCanPublisher
{
private:
	//ホイール、ハンドル関連
	/*const double handle_angle_right_max = 730;
	const double handle_angle_left_max = 765;
	const double wheelrad_to_steering_can_value_left = 20935.4958411006;//20639.7444769791;//20935.4958411006;
	const double wheelrad_to_steering_can_value_right = 20791.4464661611;//20066.8329952857;//20791.4464661611;
	const double wheelrad_to_steering_can_value_left_intercept = 0;//277.358467233321;
	const double wheelrad_to_steering_can_value_right_intercept = 0;//111.715455085083;
	const double angle_magn_right = wheelrad_to_steering_can_value_right / handle_angle_right_max;
	const double angle_magn_left = wheelrad_to_steering_can_value_left / handle_angle_left_max;*/

	ros::NodeHandle nh_, p_nh_;
	ros::Subscriber sub_config_, sub_vehicle_cmd_, sub_local_waypoints_, sub_estimate_, sub_odom_;
	ros::Subscriber sub_drive_clutch_, sub_steer_clutch_, sub_pseudo_params_;

	ros::Publisher pub_can501_, pub_can502_, pub_can503_, pub_status_, pub_way_velocity_, pub_operation_;

	autoware_config_msgs::ConfigMicrobusPseudoCanPublisher config_;
	autoware_msgs::VehicleCmd vehicle_cmd_;
	autoware_msgs::Lane waypoints_;
	nav_msgs::Odometry odom_;
	ros::Rate *rate_;//処理ステップ数
	ros::Time prev_time_;
	double way_velocity_;
	double interface_velocity_;
	bool steer_clutch_, drive_clutch_;
	autoware_can_msgs::MicroBusPseudoParams pseudo_params_;//疑似senderノードから送られてくるactual情報

	void callbackConfig(const autoware_config_msgs::ConfigMicrobusPseudoCanPublisher &msg)
	{
		config_ = msg;
		rate_ = new ros::Rate(1.0/config_.time_step_ms);
	}

	void callbackVehicleCmd(const autoware_msgs::VehicleCmd &msg)
	{
		vehicle_cmd_ = msg;
	}

	void callbackLocalWaypoints(const autoware_msgs::Lane &msg)
	{
		waypoints_ = msg;
	}

	void callbackEstimate(const geometry_msgs::PoseWithCovarianceStamped &msg)
	{
		way_velocity_ = 0;
		interface_velocity_ = 0;
	}

	void callbackOdom(const nav_msgs::Odometry &msg)
	{
		odom_ = msg;
	}

	void callbackSteerClutch(const std_msgs::Bool &msg)
	{
		steer_clutch_ = msg.data;
	}

	void callbackDriveClutch(const std_msgs::Bool &msg)
	{
		drive_clutch_ = msg.data;
	}

	void callbackPseudoParams(const autoware_can_msgs::MicroBusPseudoParams &msg)
	{
		pseudo_params_ = msg;
	}
public:
	MicrobusPseudoCanPublisher(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
		, way_velocity_(0)
		, interface_velocity_(0)
		, steer_clutch_(true)
		, drive_clutch_(true)
	{
		pub_can501_ = nh_.advertise<autoware_can_msgs::MicroBusCan501>("/microbus/can_receive501", 1);
		pub_can502_ = nh_.advertise<autoware_can_msgs::MicroBusCan502>("/microbus/can_receive502", 1);
		pub_can503_ = nh_.advertise<autoware_can_msgs::MicroBusCan503>("/microbus/can_receive503", 1);
		pub_status_ = nh_.advertise<autoware_msgs::VehicleStatus>("/microbus/vehicle_status", 1);
		pub_way_velocity_ = nh_.advertise<std_msgs::Float64>("/microbus/pseudo_way_velocity", 1);
		pub_operation_ = nh_.advertise<autoware_can_msgs::MicroBusPseudoOperation>("/microbus/pseudo_operation", 1);

		sub_config_ = nh_.subscribe("/config/microbus_pseudo_can_publisher", 1, &MicrobusPseudoCanPublisher::callbackConfig, this);
		sub_vehicle_cmd_ = nh_.subscribe("/vehicle_cmd", 1, &MicrobusPseudoCanPublisher::callbackVehicleCmd, this);
		sub_local_waypoints_ = nh_.subscribe("/final_waypoints", 1, &MicrobusPseudoCanPublisher::callbackLocalWaypoints, this);
		sub_estimate_ = nh_.subscribe("/initialpose", 1, &MicrobusPseudoCanPublisher::callbackEstimate, this);
		sub_odom_ = nh_.subscribe("/vehicle/odom", 1, &MicrobusPseudoCanPublisher::callbackOdom, this);
		sub_drive_clutch_ = nh_.subscribe("/microbus/drive_clutch", 10, &MicrobusPseudoCanPublisher::callbackDriveClutch, this);
		sub_steer_clutch_ = nh_.subscribe("/microbus/steer_clutch", 10, &MicrobusPseudoCanPublisher::callbackSteerClutch, this);
		sub_pseudo_params_ = nh_.subscribe("/microbus/pseudo_params", 10, &MicrobusPseudoCanPublisher::callbackPseudoParams, this);

		rate_ = new ros::Rate(100);
	}

	~MicrobusPseudoCanPublisher()
	{
		delete rate_;
	}

	int run_counter_ = 0;
	void run()
	{
		ros::Time nowtime = ros::Time::now();
		double time_diff = (nowtime.toSec() + nowtime.toNSec() * 1E-9) - (prev_time_.toSec() + prev_time_.toNSec() * 1E-9);

		/*if(waypoints_.waypoints.size() < 1) way_velocity_ = 0;
		else
		{
			double vel1 = waypoints_.waypoints[1].twist.twist.linear.x;//, vel2 = waypoints_.waypoints[3].twist.twist.linear.x;
			if(way_velocity_ < vel1)
			{
				way_velocity_ += 0.01;
			}
			else if(way_velocity_ > vel1)
			{
				way_velocity_ -= 0.01;
			}
			if(way_velocity_ < 0) way_velocity_ = 0;
			std::cout << way_velocity_ << "," << vel1 << std::endl;
		}*/

		if(waypoints_.waypoints.size() < 1 || drive_clutch_ == false) way_velocity_ = 0;
		else if(odom_.twist.twist.linear.x < waypoints_.waypoints[0].twist.twist.linear.x)
			way_velocity_ += 0.01;
		else if(odom_.twist.twist.linear.x > waypoints_.waypoints[0].twist.twist.linear.x)
			way_velocity_ -= 0.1;
		way_velocity_ = std::max(0.0, way_velocity_);
		std_msgs::Float64 way_vel;
		way_vel.data = way_velocity_;
		pub_way_velocity_.publish(way_vel);

		if(drive_clutch_ == false) interface_velocity_ = 0;
		else if(odom_.twist.twist.linear.x < vehicle_cmd_.ctrl_cmd.linear_velocity)
			interface_velocity_ += 0.01;
		else if(odom_.twist.twist.linear.x > vehicle_cmd_.ctrl_cmd.linear_velocity)
			interface_velocity_ -= 0.1;
		interface_velocity_ = std::max(0.0, interface_velocity_);

		autoware_can_msgs::MicroBusCan501 can501;
		can501.header.stamp = nowtime;
		can501.emergency = false;
		can501.steer_auto = autoware_can_msgs::MicroBusCan501::STEER_AUTO;
		can501.drive_auto = autoware_can_msgs::MicroBusCan501::DRIVE_AUTO;
		can501.drive_mode = autoware_can_msgs::MicroBusCan501::DRIVE_MODE_STROKE;
		can501.stroke_reply = 0;
		can501.steering_angle_reply = 0;
		can501.pedal = 0;
		can501.emergency_stop = 0;
		can501.blinker = false;
		can501.shift = autoware_can_msgs::MicroBusCan501::SHIFT_D;
		can501.read_counter = run_counter_;
		pub_can501_.publish(can501);

		autoware_can_msgs::MicroBusCan502 can502;
		can502.header.stamp = nowtime;
		can502.cycle_time = time_diff;
		/*can502.angle_deg = 0;
		if(can502.angle_deg >= 0) can502.angle_actual = can502.angle_deg / angle_magn_left;
		else can502.angle_actual = can502.angle_deg / angle_magn_right;*/
		can502.angle_actual = pseudo_params_.angle_actual * 0.1;
		if(can502.angle_actual >= 0) can502.angle_deg = can502.angle_actual * angle_magn_left;
		else can502.angle_deg = can502.angle_actual * angle_magn_right;
		if(drive_clutch_ == true)
		{
			switch(config_.use_velocity_flag)
			{
			case autoware_config_msgs::ConfigMicrobusPseudoCanPublisher::UVF_STATIC:
				//std::cout << "sta" << std::endl;
				can502.velocity_mps	= config_.velocity_mps;
				break;
			case autoware_config_msgs::ConfigMicrobusPseudoCanPublisher::UVF_WAYPOINT:
				//std::cout << "way," << way_velocity_ << std::endl;
				can502.velocity_mps = way_velocity_;
				if(can502.velocity_mps < 0.01) can502.velocity_mps = 0;
				break;
			case autoware_config_msgs::ConfigMicrobusPseudoCanPublisher::UVF_INTERFACE_CMD:
				can502.velocity_mps = interface_velocity_;
				if(can502.velocity_mps < 0.01) can502.velocity_mps = 0;
				break;
			//default:
				//std::cout << "def" << std::endl;
			}
		}
		else
		{
			//std::cout << "no" << std::endl;
			can502.velocity_mps = 0;
		}
		can502.velocity_actual = (int16_t)can502.velocity_mps * 100.0 * 3.6;
		can502.clutch = steer_clutch_;
		can502.auto_mode = true;
		can502.read_counter = run_counter_;
		pub_can502_.publish(can502);

		autoware_can_msgs::MicroBusCan503 can503;
		can503.header.stamp = nowtime;
		can503.clutch = drive_clutch_;
		can503.auto_mode = true;
		can503.pedal_displacement = (drive_clutch_ == true) ? pseudo_params_.stroke_actual : 0;
		can503.read_counter = run_counter_;
		pub_can503_.publish(can503);

		autoware_msgs::VehicleStatus status;
		status.header.stamp = nowtime;
		status.drivemode = autoware_msgs::VehicleStatus::MODE_AUTO;
		status.steeringmode = autoware_msgs::VehicleStatus::MODE_AUTO;
		status.current_gear.gear = autoware_msgs::Gear::NONE;
		status.lamp = 0;
		status.light = 0;
		status.speed = can502.velocity_mps;
		if(can502.angle_actual > 0) status.angle = can502.angle_actual / wheelrad_to_steering_can_value_left;
		else status.angle = can502.angle_actual / wheelrad_to_steering_can_value_right;
		pub_status_.publish(status);

		autoware_can_msgs::MicroBusPseudoOperation operation;
		operation.header.stamp = nowtime;
		operation.wheel_deg = status.angle;
		pub_operation_.publish(operation);

		run_counter_++;
		prev_time_ = nowtime;
		rate_->sleep();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "microbus_pseudo_can_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	MicrobusPseudoCanPublisher can_publisher(nh, private_nh);
	//ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		can_publisher.run();
		//rate.sleep();
	}
}