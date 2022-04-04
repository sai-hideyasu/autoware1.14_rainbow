#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <mobileye_560_660_msgs/ObstacleData.h>
#include <autoware_msgs/TransformMobileyeObstacleList.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class MobileyeStopper
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Subscriber sub_mobileye_obstract_;
	ros::Publisher pub_transform_mobileye_obstacle_;
	tf::TransformListener listener_;
	tf::TransformBroadcaster broadcaster_;

	double front_length_;

	//ros::Time prev_time_;
	void callbackMobileyeObstract(const mobileye_560_660_msgs::ObstacleData::ConstPtr &msg)
	{
		/*if(msg->obstacle_id == 38)
		{
			ros::Duration dt = msg->header.stamp - prev_time_;
			std::cout << dt << std::endl;
			prev_time_ = msg->header.stamp;
		}*/
		std::cout << msg->obstacle_id << "," << msg->header.stamp.nsec << std::endl;
		try
		{
			ros::Time nowtime = ros::Time::now();

			const mobileye_560_660_msgs::ObstacleData &obs = *msg;
			tf::Transform tf_detction;
			tf_detction.setOrigin(tf::Vector3(obs.obstacle_pos_x, obs.obstacle_pos_y, 0));
			tf_detction.setRotation(tf::Quaternion::getIdentity());

			std::stringstream str_b;
			str_b << "m_detect" << obs.obstacle_id;
			broadcaster_.sendTransform(tf::StampedTransform(tf_detction, nowtime, "mobileye_pos", str_b.str().c_str()));

			std::stringstream str_t;
			str_t << "m_detect" << obs.obstacle_id;
			tf::StampedTransform tf_detect;
			listener_.waitForTransform("map", str_t.str().c_str(), nowtime, ros::Duration(0.1));
			listener_.lookupTransform("map", str_t.str().c_str(), nowtime, tf_detect);

			autoware_msgs::TransformMobileyeObstacle pubdata;
			pubdata.header.stamp = msg->header.stamp;
			pubdata.pose.position.x = tf_detect.getOrigin().getX();
			pubdata.pose.position.y = tf_detect.getOrigin().getY();
			pubdata.pose.position.z = tf_detect.getOrigin().getZ();
			pubdata.pose.orientation.x = tf_detect.getRotation().getX();
			pubdata.pose.orientation.y = tf_detect.getRotation().getY();
			pubdata.pose.orientation.z = tf_detect.getRotation().getZ();
			pubdata.pose.orientation.w = tf_detect.getRotation().getW();
			pubdata.orig_data = *msg;

			pub_transform_mobileye_obstacle_.publish(pubdata);
		}
		catch(tf::TransformException ex)
		{
			std::cout << "transform error" << "," << msg->header.stamp.nsec << std::endl;
		}
	}
public:
	MobileyeStopper(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, front_length_(3.935)
	{
		
		nh_.param<double>("/vehicle_info/front_bumper_to_baselink", front_length_, 3.935);
		pub_transform_mobileye_obstacle_ = nh_.advertise<autoware_msgs::TransformMobileyeObstacle>("/transform_mobileye_obstacle", 1);
		sub_mobileye_obstract_ = nh_.subscribe("/parsed_tx/obstacle_data", 1, &MobileyeStopper::callbackMobileyeObstract, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mobileye_transform");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	MobileyeStopper stopper(nh, pnh);
	ros::spin();
	return 0;
}