#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/NearOncomingObs.h>
#include <mobileye_560_660_msgs/ObstacleData.h>
#include <autoware_msgs/TransformMobileyeObstacle.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class MobileyeStopper
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Subscriber sub_mobileye_obstract_, sub_current_pose_, sub_current_velocity_;
	ros::Publisher pub_transform_mobileye_obstacle_, pub_oncoming_;
	ros::Timer timer_;
	tf::TransformBroadcaster broadcaster_;

	double front_length_;
	bool getparam_flag_;
	Eigen::Matrix4d transform_btom_;
	Eigen::Affine3d affine_btomo_;
	bool oncoming_;//対向車が存在するか？
	geometry_msgs::Point oncoming_map_pos_;//一番近い対向車のmap位置
	double oncoming_x_;//一番近い対向車の相対位置
	ros::Time oncoming_time_;//対向車判定時間
	ros::Duration oncoming_time_th_;//対向車判定時間と現在の時間との差分がこの数値以下ならoncoming_はfalse

	//ros::Time prev_time_;
	void callbackMobileyeObstract(const mobileye_560_660_msgs::ObstacleData::ConstPtr &msg)
	{
		//std::cout << msg->obstacle_id << "," << msg->header.stamp.nsec << std::endl;
		try
		{
			ros::Time rosnowtime = ros::Time::now();
			//double nowtime = rosnowtime.sec + rosnowtime.nsec * 1E-9;
			//std::cout << std::fixed << std::setprecision(12) << nowtime << std::endl;

			const mobileye_560_660_msgs::ObstacleData &obs = *msg;

			Eigen::Vector3d vec_obs(obs.obstacle_pos_x, obs.obstacle_pos_y, 0);
			Eigen::Vector3d vec_base_to_obs = affine_btomo_ * vec_obs;

			Eigen::Translation3d tl_cur(current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
			Eigen::Quaterniond qua_cur(current_pose_.pose.orientation.w, current_pose_.pose.orientation.x,
			                           current_pose_.pose.orientation.y, current_pose_.pose.orientation.z);
			Eigen::Affine3d affine_cur = tl_cur * qua_cur;
			Eigen::Vector3d map_obs_pose = affine_cur * vec_base_to_obs;

			tf::Transform tf_eigen;
			tf_eigen.setOrigin(tf::Vector3(map_obs_pose[0], map_obs_pose[1], map_obs_pose[2]));
			tf_eigen.setRotation(tf::Quaternion::getIdentity());

			tf::Transform tf_detction;
			tf_detction.setOrigin(tf::Vector3(obs.obstacle_pos_x, obs.obstacle_pos_y, 0));
			tf_detction.setRotation(tf::Quaternion::getIdentity());

			std::stringstream str_b, eigen_b;
			str_b << "mob_detect" << obs.obstacle_id;
			eigen_b << "imob_detect" << obs.obstacle_id;
			broadcaster_.sendTransform(tf::StampedTransform(tf_detction, rosnowtime, "me_viz", str_b.str().c_str()));
			broadcaster_.sendTransform(tf::StampedTransform(tf_eigen, rosnowtime, "map", eigen_b.str().c_str()));

			autoware_msgs::TransformMobileyeObstacle pubdata;
			pubdata.header.stamp = ros::Time::now();//msg->header.stamp;
			pubdata.map_pose.position.x = map_obs_pose[0];//tf_detect.getOrigin().getX();
			pubdata.map_pose.position.y = map_obs_pose[1];//tf_detect.getOrigin().getY();
			pubdata.map_pose.position.z = map_obs_pose[2];//tf_detect.getOrigin().getZ();
			pubdata.map_pose.orientation.x = 0;//tf_detect.getRotation().getX();
			pubdata.map_pose.orientation.y = 0;//tf_detect.getRotation().getY();
			pubdata.map_pose.orientation.z = 0;//tf_detect.getRotation().getZ();
			pubdata.map_pose.orientation.w = 1;//tf_detect.getRotation().getW();
			pubdata.velocity_mps = current_velocity_.twist.linear.x + msg->obstacle_rel_vel_x;
			pubdata.orig_data = *msg;

			pub_transform_mobileye_obstacle_.publish(pubdata);

			//対向車判定
			if(msg->obstacle_status == 4)
			{
				oncoming_ = true;
				oncoming_time_ = ros::Time::now();
				if(oncoming_x_ > obs.obstacle_pos_x)
				{
					oncoming_map_pos_.x = map_obs_pose[0];
					oncoming_map_pos_.y = map_obs_pose[1];
					oncoming_map_pos_.z = map_obs_pose[2];
					oncoming_x_ = obs.obstacle_pos_x;
				}
			}
		}
		catch(tf::TransformException ex)
		{
			std::cout << "transform error" << "," << msg->header.stamp.nsec << std::endl;
		}
	}

	geometry_msgs::PoseStamped current_pose_;
	void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		current_pose_ = *msg;
	}

	geometry_msgs::TwistStamped current_velocity_;
	void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		current_velocity_ = *msg;
	}

	void callbackTimer(const ros::TimerEvent &e)
	{
		ros::Time ros_nowtime = ros::Time::now();
		ros::Duration time_diff = ros_nowtime - oncoming_time_;
		if(time_diff > oncoming_time_th_)
		{
			oncoming_ = false;
			oncoming_map_pos_.x = oncoming_map_pos_.y = DBL_MAX;
			oncoming_map_pos_.z = 0;
			oncoming_x_ = DBL_MAX;
		}
		autoware_msgs::NearOncomingObs msg;
		msg.header.stamp = ros_nowtime;
		msg.existence = oncoming_;
		msg.map_pos = oncoming_map_pos_;
		pub_oncoming_.publish(msg);
		//std_msgs::Bool oncoming_msg;
		//oncoming_msg.data = oncoming_;
		//pub_oncoming_.publish(oncoming_msg);
		//std::cout << time_diff.sec + time_diff.nsec * 1E-9 << std::endl;
	}
public:
	MobileyeStopper(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, front_length_(3.935)
		, getparam_flag_(true)
		, oncoming_(false)
		, oncoming_time_(ros::Time(0))
		, oncoming_time_th_(1.0)
	{
		double mox, moy, moz, moyaw, mopitch, moroll;
		if (nh.getParam("mox", mox) == false)
		{
			std::cout << "mox is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("moy", moy) == false)
		{
			std::cout << "moy is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("moz", moz) == false)
		{
			std::cout << "moz is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("moyaw", moyaw) == false)
		{
			std::cout << "moyaw is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("mopitch", mopitch) == false)
		{
			std::cout << "mopitch is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("moroll", moroll) == false)
		{
			std::cout << "moroll is not set." << std::endl; getparam_flag_ = false;
		}
		if(getparam_flag_ == false) return; 
		Eigen::Translation3d tl_btom(mox, moy, moz);                 // tl: translation
		Eigen::AngleAxisd rot_x_btom(moroll, Eigen::Vector3d::UnitX());  // rot: rotation
		Eigen::AngleAxisd rot_y_btom(mopitch, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z_btom(moyaw, Eigen::Vector3d::UnitZ());
		transform_btom_ = (tl_btom * rot_z_btom * rot_y_btom * rot_x_btom).matrix();
		affine_btomo_ = (tl_btom * rot_z_btom * rot_y_btom * rot_x_btom);

		nh_.param<double>("/vehicle_info/front_bumper_to_baselink", front_length_, 3.935);
		pub_transform_mobileye_obstacle_ = nh_.advertise<autoware_msgs::TransformMobileyeObstacle>("/transform_mobileye_obstacle", 1);
		pub_oncoming_ = nh_.advertise<autoware_msgs::NearOncomingObs>("/oncoming_obs", 1);

		sub_mobileye_obstract_ = nh_.subscribe<mobileye_560_660_msgs::ObstacleData>(
			"/parsed_tx/obstacle_data", 1, &MobileyeStopper::callbackMobileyeObstract, this);
		sub_current_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>(
			"/current_pose", 1, &MobileyeStopper::callbackCurrentPose, this);
		sub_current_velocity_ = nh_.subscribe<geometry_msgs::TwistStamped>(
			"/current_velocity", 1, &MobileyeStopper::callbackCurrentVelocity, this);

		timer_ = nh_.createTimer(ros::Duration(0.1), &MobileyeStopper::callbackTimer, this);

		oncoming_map_pos_.x = oncoming_map_pos_.y = DBL_MAX;
		oncoming_map_pos_.z = 0;
		oncoming_x_ = DBL_MAX;
	}

	bool isOK() {return getparam_flag_;}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mobileye_transform");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	MobileyeStopper stopper(nh, pnh);
	if(stopper.isOK() == false) return -1;
	ros::spin();
	return 0;
}