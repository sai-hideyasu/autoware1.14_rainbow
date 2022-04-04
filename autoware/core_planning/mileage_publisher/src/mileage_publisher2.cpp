#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/GnssSurfaceSpeed.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_msgs/AutoMileage.h>
#include <autoware_config_msgs/ConfigMileagePublisher.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <limits.h>

class MileagePublisher
{
private:
	ros::NodeHandle nh_, pnh_;

	ros::Subscriber sub_can502_, sub_can503_, sub_current_pose_, sub_log_folder_;
	ros::Timer checkTimer_;

	autoware_can_msgs::MicroBusCan502 can502_;//リレーボード返答 502 クラッチの状態で記録のON,OFFをする
	void callbackMicrobusCan502(const autoware_can_msgs::MicroBusCan502::ConstPtr &msg)
	{
		can502_ = *msg;
	}

	autoware_can_msgs::MicroBusCan503 can503_;//リレーボード返答 503 クラッチの状態で記録のON,OFFをする
	void callbackMicrobusCan503(const autoware_can_msgs::MicroBusCan503::ConstPtr &msg)
	{
		can503_ = *msg;
	}

	geometry_msgs::PoseStamped current_pose_;//current_poseから走行距離を計算
	void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		current_pose_ = *msg;
	}

	geometry_msgs::PoseStamped prev_current_pose_;//前回参照されたcurrent_pose
	autoware_can_msgs::MicroBusCan502 prev_can_502_;//前回参照されたcan_502
	autoware_can_msgs::MicroBusCan503 prev_can_503_;//前回参照されたcan_503
	bool record_flag_;//現在記録中かのフラグ
	double current_pose_mileage_;//current_poseでの走行距離
	ros::Time record_start_time_;//記録開始時間 書き込みファイル名に記録開始時間を入れるため
	void callbackTimer(const ros::TimerEvent& e)//定時処理用のタイマーコールバック
	{
		//can返答が1秒以上なかった場合はclutchが切られていると判断する
		ros::Time nowtime = ros::Time::now();
		ros::Duration can502_time_dt = nowtime - can502_.header.stamp;
		if(can502_time_dt > ros::Duration(1)) can502_.clutch = false;
		ros::Duration can503_time_dt = nowtime - can503_.header.stamp;
		if(can503_time_dt > ros::Duration(1)) can503_.clutch = false;

		//記録判定をclutchの状態で行う
		if(can502_.clutch == true || can503_.clutch == true)
		{
			if(record_flag_ == false) record_start_time_ = nowtime;//記録開始時の時間を保存
			record_flag_ = true;
		}
		else if(can502_.clutch == false && can503_.clutch == false)
		{
			if(record_flag_  == true) recordWrite(nowtime);//記録終了時にファイルに走行距離を記録
			record_flag_ = false;
			current_pose_mileage_ = 0;//走行距離を初期化
		}

		if(record_flag_ == true)//記録中の処理
		{
			double x = prev_current_pose_.pose.position.x - current_pose_.pose.position.x;
			double y = prev_current_pose_.pose.position.y - current_pose_.pose.position.y;
			double z = prev_current_pose_.pose.position.z - current_pose_.pose.position.z;
			double dt = sqrt(x*x + y*y + z*z);
			if(dt <= 10.0) current_pose_mileage_ += dt;//前回と今回との距離差がありえない数値(10m以上)の場合は、異常値として弾く
		}
		//std::cout << current_pose_mileage_ << "," << (int)can502_.clutch << "," << (int)can503_.clutch << std::endl;

		prev_current_pose_ = current_pose_;
		prev_can_502_ = can502_;
		prev_can_503_ = can503_;
	}

	//走行距離をファイルに出力する
	const void recordWrite(ros::Time nowtime)
	{
		//時間をdate型の文字列にする ファイル名に付与
		time_t t = nowtime.sec;
		char date[64];
		strftime(date, sizeof(date), "%Y-%m-%d-%a-%H-%M-%S", localtime(&t));

		std::string filename = log_folder_ + "/mileage_" + date + ".csv";

		//走行距離をファイルに出力
		std::ofstream ofs(filename, std::ios_base::out);
		ofs << "current_pose" << std::endl;
		ofs << std::fixed << std::setprecision(6) <<  current_pose_mileage_;
		ofs.close();
	}

	std::string log_folder_;//走行距離ファイルの保存フォルダ
	void callbackLogFolder(const std_msgs::String::ConstPtr &msg)
	{
		log_folder_ = msg->data;
	}
public:
	MileagePublisher(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, record_flag_(false)
		, current_pose_mileage_(0)
	{
		sub_can502_ = nh_.subscribe("/microbus/can_receive502", 10, &MileagePublisher::callbackMicrobusCan502, this);
		sub_can503_ = nh_.subscribe("/microbus/can_receive503", 10, &MileagePublisher::callbackMicrobusCan503, this);
		sub_current_pose_ = nh_.subscribe("/current_pose", 10, &MileagePublisher::callbackCurrentPose, this);
		sub_log_folder_ = nh_.subscribe("/microbus/log_folder", 10, &MileagePublisher::callbackLogFolder, this);
		checkTimer_ = nh.createTimer(ros::Duration(0.1), &MileagePublisher::callbackTimer, this);

		prev_can_502_.clutch = false;
		prev_can_503_.clutch = false;
		log_folder_ = ros::package::getPath("runtime_manager");//デフォルトの保存場所はruntime_managerパッケージ
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mileage_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	MileagePublisher mp(nh, pnh);
	ros::spin();
	return 0;
}