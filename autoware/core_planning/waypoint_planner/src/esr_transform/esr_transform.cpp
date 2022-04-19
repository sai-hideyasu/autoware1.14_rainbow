#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/TransformEsrObstacleList.h>
#include <derived_object_msgs/ObjectWithCovarianceArray.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class EsrTransform
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber sub_esr_objects_, sub_current_pose_, sub_current_velocity_;
	ros::Publisher pub_transform_esr_obstacles_;
	//tf::TransformBroadcaster broadcaster_;

	double front_length_;
	bool getparam_flag_;
	Eigen::Matrix4d transform_btom_;
	Eigen::Affine3d affine_btomo_;

	void callbackEsrObjects(const derived_object_msgs::ObjectWithCovarianceArray::ConstPtr &msg)
	{
		ros::Time rosnowtime = ros::Time::now();
		//double nowtime = rosnowtime.sec + rosnowtime.nsec * 1E-9;
		//std::cout << std::fixed << std::setprecision(6) << nowtime << std::endl;

		autoware_msgs::TransformEsrObstacleList pubmsg;
		pubmsg.header.stamp = rosnowtime;

		for(const derived_object_msgs::ObjectWithCovariance &esr : msg->objects)
		{
			Eigen::Vector3d vec_obs(esr.pose.pose.position.x, esr.pose.pose.position.y, esr.pose.pose.position.z);
			Eigen::Vector3d vec_base_to_obs = affine_btomo_ * vec_obs;

			Eigen::Translation3d tl_cur(current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
			Eigen::Quaterniond qua_cur(current_pose_.pose.orientation.w, current_pose_.pose.orientation.x,
			                           current_pose_.pose.orientation.y, current_pose_.pose.orientation.z);
			Eigen::Affine3d affine_cur = tl_cur * qua_cur;
			Eigen::Vector3d map_obs_pose = affine_cur * vec_base_to_obs;

			autoware_msgs::TransformEsrObstacle obsmsg;
			obsmsg.header.stamp = rosnowtime;
			obsmsg.map_pose.position.x = map_obs_pose[0];//tf_detect.getOrigin().getX();
			obsmsg.map_pose.position.y = map_obs_pose[1];//tf_detect.getOrigin().getY();
			obsmsg.map_pose.position.z = map_obs_pose[2];//tf_detect.getOrigin().getZ();
			obsmsg.map_pose.orientation.x = 0;
			obsmsg.map_pose.orientation.y = 0;
			obsmsg.map_pose.orientation.z = 0;
			obsmsg.map_pose.orientation.w = 1;
			obsmsg.velocity_mps = esr.twist.twist;
			obsmsg.velocity_mps.linear.x += current_velocity_.twist.linear.x;
			obsmsg.velocity_mps.linear.y += current_velocity_.twist.linear.y;
			obsmsg.orig_data = esr;
			pubmsg.obstacles.push_back(obsmsg);

			/*tf::Transform tf_eigen;
			tf_eigen.setOrigin(tf::Vector3(map_obs_pose[0], map_obs_pose[1], map_obs_pose[2]));
			tf_eigen.setRotation(tf::Quaternion::getIdentity());

			tf::Transform tf_detction;
			tf_detction.setOrigin(tf::Vector3(esr.pose.pose.position.x, esr.pose.pose.position.y, esr.pose.pose.position.z));
			tf_detction.setRotation(tf::Quaternion::getIdentity());

			std::stringstream str_b, eigen_b;
			str_b << "esr_detect" << esr.id;
			eigen_b << "iesr_detect" << esr.id;
			broadcaster_.sendTransform(tf::StampedTransform(tf_detction, nowtime, "esr_1", str_b.str().c_str()));
			broadcaster_.sendTransform(tf::StampedTransform(tf_eigen, nowtime, "map", eigen_b.str().c_str()));*/
		}

		pub_transform_esr_obstacles_.publish(pubmsg);
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
public:
	EsrTransform(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, getparam_flag_(true)
	{
		double esrx, esry, esrz, esryaw, esrpitch, esrroll;
		if (nh.getParam("esrx", esrx) == false)
		{
			std::cout << "esrx is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("esry", esry) == false)
		{
			std::cout << "esry is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("esrz", esrz) == false)
		{
			std::cout << "esrz is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("esryaw", esryaw) == false)
		{
			std::cout << "esryaw is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("esrpitch", esrpitch) == false)
		{
			std::cout << "esrpitch is not set." << std::endl; getparam_flag_ = false;
		}
		if (nh.getParam("esrroll", esrroll) == false)
		{
			std::cout << "esrroll is not set." << std::endl; getparam_flag_ = false;
		}
		if(getparam_flag_ == false) return; 
		Eigen::Translation3d tl_btom(esrx, esry, esrz);                 // tl: translation
		Eigen::AngleAxisd rot_x_btom(esrroll, Eigen::Vector3d::UnitX());  // rot: rotation
		Eigen::AngleAxisd rot_y_btom(esrpitch, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rot_z_btom(esryaw, Eigen::Vector3d::UnitZ());
		transform_btom_ = (tl_btom * rot_z_btom * rot_y_btom * rot_x_btom).matrix();
		affine_btomo_ = (tl_btom * rot_z_btom * rot_y_btom * rot_x_btom);

		nh_.param<double>("/vehicle_info/front_bumper_to_baselink", front_length_, 3.935);
		pub_transform_esr_obstacles_ = nh_.advertise<autoware_msgs::TransformEsrObstacleList>("/transform_esr_obstacles", 1);

		sub_esr_objects_ = nh_.subscribe<derived_object_msgs::ObjectWithCovarianceArray>(
			"/esr/as_tx/objects", 1, &EsrTransform::callbackEsrObjects, this);
		sub_current_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>(
			"/current_pose", 1, &EsrTransform::callbackCurrentPose, this);
		sub_current_velocity_ = nh_.subscribe<geometry_msgs::TwistStamped>(
			"/current_velocity", 1, &EsrTransform::callbackCurrentVelocity, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "esr_transform");
	ros::NodeHandle nh, pnh("~");

	EsrTransform esr(nh, pnh);
	ros::spin();
	return 0;
}