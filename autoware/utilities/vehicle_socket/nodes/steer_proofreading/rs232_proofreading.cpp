#include <ros/ros.h>
#include <fstream>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include <autoware_can_msgs/MicroBusSHHV.h>
#include <autoware_can_msgs/SteerProofreading.h>

class RS232Proofreading
{
private:
	//rs232SteerMainsSlop 21.962
	//rs232SteerMainsInc -23148.6
	//中央値 960
	//steer_actual 21.962 * 960 + (-23148.6) = (-2065.08)
	//rs232_proofreadingで計算した倍率はwheelrad_to_steering_can_value_left_とwheelrad_to_steering_can_value_right_にかける
	const double VELTAGE_TO_ACTUAL_SLOP = 21.962;
	const double VELTAGE_TO_ACTUAL_INC = -23148.6;

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Publisher pub_steer_proofreading_main_;
	ros::Publisher pub_steer_proofreading_base_;
	ros::Subscriber sub_config_;
	ros::Subscriber sub_steer_proofreading_info_;
	ros::Subscriber sub_shhv_;

	int use_voltage_;//mainとsubのどちらをキャリブレーションするか？
	int8_t steer_proofreading_info_;//steer_proofreadingから送られてくる現在のキャリブレーション処理段階の数値
	std::vector<autoware_can_msgs::MicroBusSHHV> shhv_list_;//RS232通信から送られてくるJOYシステムのハンドルステータス
	std::vector<double> time_list_;
	bool end_flag_;

	void proofreading()
	{
		std::ofstream ofs("/tmp/steer_proofreading.csv");
		for(int i=0; i<shhv_list_.size(); i++)
		{
			ofs << std::fixed << std::setprecision(5) << shhv_list_[i].joy_angle_main << "," << shhv_list_[i].joy_angle_sub
				<< "," << shhv_list_[i].mechanism_steering_main <<  "," << shhv_list_[i].mechanism_steering_sub
				<< "," << time_list_[i] << std::endl;
		}
		ofs.close();

		switch(use_voltage_)
		{
		case 0://サブの電圧レンジ及び直進箇所の電圧値を設定
			{
				autoware_can_msgs::SteerProofreading msg;
				//サブ電圧の最大と最小を取得
				msg.min_voltage_sub = USHRT_MAX, msg.max_voltage_sub = 0;
				msg.min_voltage_main = USHRT_MAX, msg.max_voltage_main = 0;
				for(autoware_can_msgs::MicroBusSHHV shhv : shhv_list_)
				{
					msg.min_voltage_sub = std::min(msg.min_voltage_sub, shhv.mechanism_steering_sub);
					msg.max_voltage_sub = std::max(msg.max_voltage_sub, shhv.mechanism_steering_sub);
					msg.min_voltage_main = std::min(msg.min_voltage_main, shhv.mechanism_steering_main);
					msg.max_voltage_main = std::max(msg.max_voltage_main, shhv.mechanism_steering_main);
				}
				pub_steer_proofreading_base_.publish(msg);
			}
			break;
		case 1://メイン電圧処理
			break;
		}

		//最小二乗法を用いて倍率と中央値を計算
		/*uint16_t xmin=USHRT_MAX, xmax=0;
		double xave=0, yave=0;
		for(const autoware_can_msgs::MicroBusSHHV shhv : shhv_list_)
		{
			xave += shhv.mechanism_steering_sub;
			yave += shhv.mechanism_steering_main;
			xmin = std::min(xmin, shhv.mechanism_steering_sub);
			xmax = std::max(xmax, shhv.mechanism_steering_sub);
		}
		xave /= shhv_list_.size();
		yave /= shhv_list_.size();
		double sum1=0, sum2=0;
		for(const autoware_can_msgs::MicroBusSHHV shhv : shhv_list_)
		{
			sum1 += (shhv.mechanism_steering_sub - xave) * (shhv.mechanism_steering_main - yave);
			sum2 += (shhv.mechanism_steering_sub - xave) * (shhv.mechanism_steering_sub - xave);
		}
		double slop = sum1 / sum2;
		double inc = yave - slop * xave;

		//sub側の最大値と最小値(y値)の中点と同じy値となるmain値(x値)
		double xcenter = (xmax + xmin) / 2.0;
		double voltage_center = slop * xcenter + inc;

		autoware_can_msgs::SteerProofreading msg_value;
		msg_value.header.stamp = ros::Time::now();
		msg_value.magn = slop;
		msg_value.actual_center = VELTAGE_TO_ACTUAL_SLOP * voltage_center + VELTAGE_TO_ACTUAL_INC;
		pub_steer_proofreading_main_.publish(msg_value);*/

		end_flag_ = true;
		sub_steer_proofreading_info_.shutdown();
	}

	void callbackSteerProofreadingInfo(const std_msgs::Int8::ConstPtr &msg)
	{
		steer_proofreading_info_ = msg->data;
		if(steer_proofreading_info_ < 0) proofreading();
	}

	void callbackSHHV(const autoware_can_msgs::MicroBusSHHV::ConstPtr &msg)
	{
		if(steer_proofreading_info_ == 3)
		{
			shhv_list_.push_back(*msg);
			ros::Time ros_nowtime = ros::Time::now();
			double nowtime = ros_nowtime.sec + ros_nowtime.nsec * 1E-9;
			time_list_.push_back(nowtime);
		}
	}
public:
	RS232Proofreading(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, steer_proofreading_info_(0)
		, end_flag_(false)
	{
		use_voltage_ = nh_.param<int>("use_voltage", 0);

		pub_steer_proofreading_main_ = nh_.advertise<autoware_can_msgs::SteerProofreading>("/steer_proofreading/main", 1, false);
		pub_steer_proofreading_base_ = nh_.advertise<autoware_can_msgs::SteerProofreading>("/steer_proofreading/base", 1, false);
		sub_steer_proofreading_info_ = nh_.subscribe<std_msgs::Int8>
			("/steer_proofreading/info", 1, &RS232Proofreading::callbackSteerProofreadingInfo, this);
		sub_shhv_ = nh_.subscribe<autoware_can_msgs::MicroBusSHHV>
			("/microbus/shhv", 1, &RS232Proofreading::callbackSHHV, this);
	}

	bool end() {return end_flag_;}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rs232_proofreading");
	ros::NodeHandle nh, pnh("~");

	RS232Proofreading rs(nh, pnh);
	ros::Rate rate1(ros::Duration(1.0 / 100.0));
	while(ros::ok())
	{
		ros::spinOnce();
		if(rs.end()) break;
		rate1.sleep();
	}
	ros::Rate rate2(ros::Duration(0.01));
	for(int counter=0; counter<100 && ros::ok(); counter++)
	{
		rate2.sleep();
	}
	return 0;
}