#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_msgs/GnssSurfaceSpeed.h>
#include <autoware_msgs/GnssToBaselink.h>
#include <autoware_config_msgs/ConfigGnssLocalizer.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/LocalizerCorrect.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, autoware_msgs::GnssSurfaceSpeed>
	GnssSync;

//pthread_mutex_t mutex;//スレッドのロックに使用する予定

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	  tf::Quaternion quat;
	  quaternionMsgToTF(geometry_quat, quat);
	  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

geometry_msgs::PoseStamped position_correct(const geometry_msgs::PoseStamped &pose, const double y_correct)
{
	Eigen::Vector3d po(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	double roll, pitch, yaw;
	geometry_quat_to_rpy(roll, pitch, yaw, pose.pose.orientation);
	Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_rot = qua * po;
	po_rot += Eigen::Vector3d(0, y_correct, 0);
	Eigen::Quaterniond qua_rev = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_move = qua_rev * po_rot;

	geometry_msgs::PoseStamped ret;
	ret.header = pose.header;
	ret.pose.position.x = po_move.x();
	ret.pose.position.y = po_move.y();
	ret.pose.position.z = po_move.z();
	ret.pose.orientation = pose.pose.orientation;
	return ret;
}

struct pose
{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};

class gnss_matching_class
{
private:
	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	Eigen::Matrix4f tf_btog, tf_btov;

	ros::Publisher pub_localizer_pose_;
	ros::Publisher pub_estimate_twist_;
	ros::Publisher pub_gnss_pose_vehicle_;
	ros::Publisher pub_correct_;
	ros::Publisher pub_tmp_;

	ros::Subscriber sub_config_, sub_gnss_to_baselink_, sub_waypoint_param_;
	ros::Subscriber sub_localizer_correct_;//!< localizer_smooth_transtionノードから送られる補正情報
  
	tf::TransformBroadcaster gnss_tb;

	autoware_config_msgs::ConfigGnssLocalizer config_;
	double yaw_correct_deg_;//!< 向きの補正値
	double y_correct_;//!< y方向の補正値
	geometry_msgs::PoseStamped prev_pose_;

	void correctPublish()
	{
		autoware_msgs::LocalizerCorrect cor_msg;
		cor_msg.header.stamp = ros::Time::now();
		cor_msg.distance_correct = y_correct_;
		cor_msg.angle_correct_deg = yaw_correct_deg_;
		pub_correct_.publish(cor_msg);
	}

	void callbackLocalizerCorrect(const autoware_msgs::LocalizerCorrect &msg)
	{
		if(msg.angle_correct_deg >= -autoware_msgs::LocalizerCorrect::ANGLE_CORRECT_DEG_TH &&
			msg.angle_correct_deg <= autoware_msgs::LocalizerCorrect::ANGLE_CORRECT_DEG_TH)
		{
			yaw_correct_deg_ = msg.angle_correct_deg;
		}
		if(msg.distance_correct >= -autoware_msgs::LocalizerCorrect::DISTANCE_CORRECT_TH &&
			msg.distance_correct <= autoware_msgs::LocalizerCorrect::DISTANCE_CORRECT_TH)
		{
			y_correct_ = msg.distance_correct;
		}
		correctPublish();
	}

	void callbackConfig(const autoware_config_msgs::ConfigGnssLocalizer &msg)
	{
		if(msg.yaw_correct_deg >= -autoware_msgs::LocalizerCorrect::ANGLE_CORRECT_DEG_TH &&
			msg.yaw_correct_deg <= autoware_msgs::LocalizerCorrect::ANGLE_CORRECT_DEG_TH)
		{
			yaw_correct_deg_ = msg.yaw_correct_deg;
		}
		if(msg.position_y_correct >= -autoware_msgs::LocalizerCorrect::DISTANCE_CORRECT_TH &&
			msg.position_y_correct <= autoware_msgs::LocalizerCorrect::DISTANCE_CORRECT_TH)
		{
			y_correct_ = msg.position_y_correct;
		}
		config_ = msg;
		correctPublish();
	}
  
	void callbackWaypointParam(const autoware_msgs::WaypointParam &msg)
	{
		if(msg.gnss_yaw_correct_deg >= -autoware_msgs::LocalizerCorrect::ANGLE_CORRECT_DEG_TH &&
			msg.gnss_yaw_correct_deg <= autoware_msgs::LocalizerCorrect::ANGLE_CORRECT_DEG_TH)
		{
			yaw_correct_deg_ = msg.gnss_yaw_correct_deg;
		}
		if(msg.gnss_y_correct >= -autoware_msgs::LocalizerCorrect::DISTANCE_CORRECT_TH &&
			msg.gnss_y_correct <= autoware_msgs::LocalizerCorrect::DISTANCE_CORRECT_TH)
		{
			y_correct_ = msg.gnss_y_correct;
		}
		correctPublish();
	}

	void callbackGnssToLocalizer(const autoware_msgs::GnssToBaselink &msg)
	{
		Eigen::Translation3f tl_btog(msg.x, msg.y, msg.z);                 // tl: translation
		Eigen::AngleAxisf rot_x_btog(msg.roll, Eigen::Vector3f::UnitX());  // rot: rotation
		Eigen::AngleAxisf rot_y_btog(msg.pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_z_btog(msg.yaw, Eigen::Vector3f::UnitZ());
		tf_btog = (tl_btog * rot_z_btog * rot_y_btog * rot_x_btog).matrix();
	}

	double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
	{
		double diff_rad = lhs_rad - rhs_rad;
		if(diff_rad >= M_PI)
			diff_rad = diff_rad - 2*M_PI;
		else if(diff_rad < -M_PI)
			diff_rad = diff_rad + 2*M_PI;
		return diff_rad;
	}

	void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
		tf::Quaternion quat;
		quaternionMsgToTF(geometry_quat, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
	}

	void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
						   const autoware_msgs::GnssSurfaceSpeed::ConstPtr& speed_msg)
	{
		ros::Time current_time = pose_msg->header.stamp;

		geometry_msgs::PoseStamped current_pose = position_correct(*pose_msg, y_correct_);
		
		tf::Quaternion gnss_q(current_pose.pose.orientation.x, current_pose.pose.orientation.y,
			current_pose.pose.orientation.z, current_pose.pose.orientation.w);
		tf::Quaternion q_correct = tf::createQuaternionFromYaw(yaw_correct_deg_ * M_PI / 180.0);
		tf::Quaternion qua = gnss_q * q_correct;
		current_pose.pose.orientation.x = qua.getX();
		current_pose.pose.orientation.y = qua.getY();
		current_pose.pose.orientation.z = qua.getZ();
		current_pose.pose.orientation.w = qua.getW();
		
		tf::Matrix3x3 gnss_m(qua);
		Eigen::Matrix4f t_gnss(Eigen::Matrix4f::Identity());   // base_link
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++) t_gnss(i,j)=gnss_m[i][j];
		}
		t_gnss(0,3)=current_pose.pose.position.x; t_gnss(1,3)=current_pose.pose.position.y; t_gnss(2,3)=current_pose.pose.position.z;

		Eigen::Matrix4f t2_gnss = t_gnss * tf_btog.inverse();
		Eigen::Matrix4f t_velodyne = t2_gnss * tf_btov;
		publish_localizer(t_velodyne, current_time);


		// Update gnss_pose2
		tf::Matrix3x3 mat_b_gnss;  // base_link
		mat_b_gnss.setValue(static_cast<double>(t2_gnss(0, 0)), static_cast<double>(t2_gnss(0, 1)), static_cast<double>(t2_gnss(0, 2)),
			static_cast<double>(t2_gnss(1, 0)), static_cast<double>(t2_gnss(1, 1)), static_cast<double>(t2_gnss(1, 2)),
			static_cast<double>(t2_gnss(2, 0)), static_cast<double>(t2_gnss(2, 1)), static_cast<double>(t2_gnss(2, 2)));
		pose gnss_pose;
		gnss_pose.x = t2_gnss(0, 3);
		gnss_pose.y = t2_gnss(1, 3);
		gnss_pose.z = t2_gnss(2, 3);
		mat_b_gnss.getRPY(gnss_pose.roll, gnss_pose.pitch, gnss_pose.yaw, 1);
		qua.setRPY(gnss_pose.roll, gnss_pose.pitch, gnss_pose.yaw);
		tf::Vector3 v(gnss_pose.x, gnss_pose.y, gnss_pose.z);
		tf::Transform transform(qua, v);
		geometry_msgs::PoseStamped gnss_pose_msg;
		gnss_pose_msg.header.frame_id = "/map";
		gnss_pose_msg.header.stamp = current_time;
		gnss_pose_msg.pose.position.x = transform.getOrigin().getX();
		gnss_pose_msg.pose.position.y = transform.getOrigin().getY();
		gnss_pose_msg.pose.position.z = transform.getOrigin().getZ();
		gnss_pose_msg.pose.orientation.x = transform.getRotation().x();
		gnss_pose_msg.pose.orientation.y = transform.getRotation().y();
		gnss_pose_msg.pose.orientation.z = transform.getRotation().z();
		gnss_pose_msg.pose.orientation.w = transform.getRotation().w();
		pub_gnss_pose_vehicle_.publish(gnss_pose_msg);

		tf::Quaternion current_q;
		transform.setOrigin(tf::Vector3(gnss_pose.x, gnss_pose.y, gnss_pose.z));
		current_q.setRPY(gnss_pose.roll, gnss_pose.pitch, gnss_pose.yaw);
		transform.setRotation(current_q);
		gnss_tb.sendTransform(tf::StampedTransform(transform, current_time, "/map", "/rtk_base_link"));

		if(prev_pose_.header.stamp != ros::Time(0))
		{
			pose gnss_pose_prev;
			gnss_pose_prev.x = prev_pose_.pose.position.x;//t2_gnss_prev(0, 3);
			gnss_pose_prev.y = prev_pose_.pose.position.y;//t2_gnss_prev(1, 3);
			gnss_pose_prev.z = prev_pose_.pose.position.z;//t2_gnss_prev(2, 3);
			geometry_quat_to_rpy(gnss_pose_prev.roll, gnss_pose_prev.pitch, gnss_pose_prev.yaw, prev_pose_.pose.orientation);
			std::cout << "current:" << gnss_pose.yaw << " ,prev:" << gnss_pose_prev.yaw << std::endl;
			publish_estimate_twist(gnss_pose, gnss_pose_prev, speed_msg->surface_speed, current_time, prev_pose_.header.stamp);
		}

		prev_pose_ = current_pose;
	}

	void publish_estimate_twist(pose current, pose prev, double gnss_speed, ros::Time current_time, ros::Time prev_time)
	{
		double diff_time = (current_time - prev_time).toSec();

		double diff_x = current.x - prev.x;
		double diff_y = current.y - prev.y;
		double diff_z = current.z - prev.z;
		double diff_yaw = calcDiffForRadian(current.yaw, prev.yaw);
		double diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

		double velocity = (diff_time > 0) ? (diff / diff_time) : 0;
		double angular_velocity   = (diff_time > 0) ? (diff_yaw / diff_time) : 0;

		geometry_msgs::TwistStamped estimate_twist_msg;
		estimate_twist_msg.header.stamp = current_time;
		estimate_twist_msg.header.frame_id = "/rtk_base_link";
		estimate_twist_msg.twist.linear.x = gnss_speed;//velocity;
		estimate_twist_msg.twist.linear.y = 0.0;
		estimate_twist_msg.twist.linear.z = 0.0;
		estimate_twist_msg.twist.angular.x = 0.0;
		estimate_twist_msg.twist.angular.y = 0.0;
		estimate_twist_msg.twist.angular.z = angular_velocity;

		pub_estimate_twist_.publish(estimate_twist_msg);
	}

	void publish_localizer(Eigen::Matrix4f t_gnss,ros::Time updateTime)
	{
		tf::Matrix3x3 mat_l;  // localizer
		mat_l.setValue(static_cast<double>(t_gnss(0, 0)), static_cast<double>(t_gnss(0, 1)), static_cast<double>(t_gnss(0, 2)),
			static_cast<double>(t_gnss(1, 0)), static_cast<double>(t_gnss(1, 1)), static_cast<double>(t_gnss(1, 2)),
			static_cast<double>(t_gnss(2, 0)), static_cast<double>(t_gnss(2, 1)), static_cast<double>(t_gnss(2, 2)));

		// Update localizer_pose
		pose localizer_pose;
		localizer_pose.x = t_gnss(0, 3);
		localizer_pose.y = t_gnss(1, 3);
		localizer_pose.z = t_gnss(2, 3);
		mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);
		//localizer_pose.yaw += M_PI;
		//if(localizer_pose.yaw >= M_PI) localizer_pose.yaw -= M_PI*2;
		tf::Quaternion localizer_q;
		localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);

		geometry_msgs::PoseStamped localizer_pose_msg;
		localizer_pose_msg.header.frame_id = "/map";
		localizer_pose_msg.header.stamp = updateTime;
		localizer_pose_msg.pose.position.x = localizer_pose.x;
		localizer_pose_msg.pose.position.y = localizer_pose.y;
		localizer_pose_msg.pose.position.z = localizer_pose.z;
		localizer_pose_msg.pose.orientation.x = localizer_q.x();
		localizer_pose_msg.pose.orientation.y = localizer_q.y();
		localizer_pose_msg.pose.orientation.z = localizer_q.z();
		localizer_pose_msg.pose.orientation.w = localizer_q.w();
		pub_localizer_pose_.publish(localizer_pose_msg);
	}
public:
	gnss_matching_class(ros::NodeHandle nh_, ros::NodeHandle private_nh_)
	  : yaw_correct_deg_(0)
	  , y_correct_(0)
	{
		prev_pose_.header.stamp = ros::Time(0);

		nh = nh_; private_nh = private_nh_;

		const unsigned int max_his_size=10;

		double _tf_vx, _tf_vy, _tf_vz, _tf_vroll, _tf_vpitch, _tf_vyaw;
		if (nh.getParam("tf_x", _tf_vx) == false)
		{
			std::cout << "tf_x is not set." << std::endl;
			return;
		}
		if (nh.getParam("tf_y", _tf_vy) == false)
		{
			std::cout << "tf_y is not set." << std::endl;
			return;
		}
		if (nh.getParam("tf_z", _tf_vz) == false)
		{
			std::cout << "tf_z is not set." << std::endl;
			return;
		}
		if (nh.getParam("tf_roll", _tf_vroll) == false)
		{
			std::cout << "tf_roll is not set." << std::endl;
			return;
		}
		if (nh.getParam("tf_pitch", _tf_vpitch) == false)
		{
			std::cout << "tf_pitch is not set." << std::endl;
			return;
		}
		if (nh.getParam("tf_yaw", _tf_vyaw) == false)
		{
			std::cout << "tf_yaw is not set." << std::endl;
			return;
		}
		Eigen::Translation3f tl_btov(_tf_vx, _tf_vy, _tf_vz);                 // tl: translation
		Eigen::AngleAxisf rot_x_btov(_tf_vroll, Eigen::Vector3f::UnitX());  // rot: rotation
		Eigen::AngleAxisf rot_y_btov(_tf_vpitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_z_btov(_tf_vyaw, Eigen::Vector3f::UnitZ());
		tf_btov = (tl_btov * rot_z_btov * rot_y_btov * rot_x_btov).matrix();

		pub_localizer_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/gnss_localizer_pose", 10);
		pub_estimate_twist_ = nh.advertise<geometry_msgs::TwistStamped>("/gnss_estimate_twist", 10);
		pub_gnss_pose_vehicle_ = nh.advertise<geometry_msgs::PoseStamped>("/RTK_gnss_pose", 10);
		pub_correct_ = nh.advertise<autoware_msgs::LocalizerCorrect>("/gnss_localizer_correct", 10, true);
		//pub_tmp_ = nh.advertise<std_msgs::String>("/RTK_tmp", 10);

		sub_config_ = nh.subscribe("/config/gnss_localizer", 1, &gnss_matching_class::callbackConfig, this);
		sub_gnss_to_baselink_ = nh.subscribe("/gnss_to_base_link", 1, &gnss_matching_class::callbackGnssToLocalizer, this);
		sub_waypoint_param_ = nh.subscribe("/waypoint_param", 1, &gnss_matching_class::callbackWaypointParam, this);
		sub_localizer_correct_ = nh.subscribe("/smooth_transition_correct_gnss", 1, &gnss_matching_class::callbackLocalizerCorrect, this);
		message_filters::Subscriber<geometry_msgs::PoseStamped> sub_gnss_pose(nh, "gnss_pose", 10);
		message_filters::Subscriber<autoware_msgs::GnssSurfaceSpeed> sub_surface_speed(nh, "gnss_surface_speed", 10);
		message_filters::Synchronizer<GnssSync> sync_gnss(GnssSync(10),sub_gnss_pose, sub_surface_speed);
		sync_gnss.registerCallback(boost::bind(&gnss_matching_class::gnss_callback, this, _1, _2));

		correctPublish();

		ros::spin();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gnss_matching");
//    pthread_mutex_init(&mutex, NULL);

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	gnss_matching_class gmc(nh,private_nh);
	return 0;
}
