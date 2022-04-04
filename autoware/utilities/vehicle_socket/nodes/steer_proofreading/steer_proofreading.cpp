#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <autoware_msgs/SteerOverride.h>
#include <autoware_can_msgs/MicroBusSHHV.h>

class SteerProofreading
{
private:
	const static double STEER_OVERRIDE_TH = -100000;//steerのactualを上書きする判定のしきい値
	const int16_t MAX_502_STEER_CMD = 19000;
	const int16_t MIN_502_STEER_CMD = -19000;
	const int16_t ERECTRIC_CURRENT_TH_ABS = 200;

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber sub_shhv_;
	ros::Publisher pub_steer_override_;
	ros::Publisher pub_steer_proofreading_info_;
	ros::Timer timer_;

	autoware_can_msgs::MicroBusSHHV shhv_;
	bool end_flag_;

	void callbackSHHV(const autoware_can_msgs::MicroBusSHHV::ConstPtr &msg)
	{
		shhv_ = *msg;
	}
public:
	SteerProofreading(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, end_flag_(false)
	{
		pub_steer_override_ = nh_.advertise<autoware_msgs::SteerOverride>("/microbus/steer_override", 1, false);
		pub_steer_proofreading_info_ = nh_.advertise<std_msgs::Int8>("/steer_proofreading/info", 1, false);
		sub_shhv_ = nh_.subscribe<autoware_can_msgs::MicroBusSHHV>("/microbus/shhv", 1, &SteerProofreading::callbackSHHV, this);
		timer_ = nh_.createTimer(ros::Duration(3.0), &SteerProofreading::callbackTimer, this);
	}

	void callbackTimer(const ros::TimerEvent &e)
	{
		{
			std_msgs::Int8 msg_steer_proofreading;
			msg_steer_proofreading.data = 1;
			pub_steer_proofreading_info_.publish(msg_steer_proofreading);
			autoware_msgs::SteerOverride msg_steer_value;
			msg_steer_value.steer_value = 0;
			pub_steer_override_.publish(msg_steer_value);
			ros::Rate rate(ros::Duration(3.0));
			rate.sleep();
		}
		{
			for(double s=0; s<=MAX_502_STEER_CMD; s+=100)
			{
				if(std::abs(shhv_.motor_electric_current) > ERECTRIC_CURRENT_TH_ABS)
					break;

				std_msgs::Int8 msg_steer_proofreading;
				msg_steer_proofreading.data = 2;
				pub_steer_proofreading_info_.publish(msg_steer_proofreading);
				autoware_msgs::SteerOverride msg_steer_value;
				msg_steer_value.steer_value = s;
				pub_steer_override_.publish(msg_steer_value);
				ros::Rate rate(ros::Duration(0.03));
				rate.sleep();
			}
			ros::Rate rate(ros::Duration(3.0));
			rate.sleep();
		}

		{
			for(double s=MAX_502_STEER_CMD; s>0; s-=100)
			{
				std_msgs::Int8 msg_steer_proofreading;
				msg_steer_proofreading.data = 3;
				pub_steer_proofreading_info_.publish(msg_steer_proofreading);
				autoware_msgs::SteerOverride msg_steer_value;
				msg_steer_value.steer_value = s;
				pub_steer_override_.publish(msg_steer_value);
				ros::Rate rate(ros::Duration(0.03));
				rate.sleep();
			}
		}

		{
			for(double s=0; s>=MIN_502_STEER_CMD; s-=100)
			{
				if(std::abs(shhv_.motor_electric_current) > ERECTRIC_CURRENT_TH_ABS)
					break;

				std_msgs::Int8 msg_steer_proofreading;
				msg_steer_proofreading.data = 3;
				pub_steer_proofreading_info_.publish(msg_steer_proofreading);
				autoware_msgs::SteerOverride msg_steer_value;
				msg_steer_value.steer_value = s;
				pub_steer_override_.publish(msg_steer_value);
				ros::Rate rate(ros::Duration(0.03));
				rate.sleep();
			}
			ros::Rate rate(ros::Duration(1.0));
			rate.sleep();
		}

		{
			std_msgs::Int8 msg_steer_proofreading;
			msg_steer_proofreading.data = 4;
			pub_steer_proofreading_info_.publish(msg_steer_proofreading);
			autoware_msgs::SteerOverride msg_steer_value;
			msg_steer_value.steer_value = 0;
			pub_steer_override_.publish(msg_steer_value);
			ros::Rate rate(ros::Duration(5.0));
			rate.sleep();
		}

		{
			std_msgs::Int8 msg_steer_proofreading;
			msg_steer_proofreading.data = -1;
			pub_steer_proofreading_info_.publish(msg_steer_proofreading);
			autoware_msgs::SteerOverride msg_steer_value;
			msg_steer_value.steer_value = STEER_OVERRIDE_TH;
			pub_steer_override_.publish(msg_steer_value);
			ros::Rate rate(ros::Duration(1.0));
			rate.sleep();
		}

		timer_.stop();
		end_flag_ = true;
	}

	bool end(){return end_flag_;}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "steer_proofreading");
	ros::NodeHandle nh, pnh("~");

	SteerProofreading pr(nh, pnh);
	while(ros::ok())
	{
		ros::spinOnce();
		if(pr.end()) break;
	}

	ros::Rate rate(ros::Duration(0.01));
	for(int counter=0; counter<100 && ros::ok(); counter++)
	{
		rate.sleep();
	}
	//ros::spin();
	return 0;
}