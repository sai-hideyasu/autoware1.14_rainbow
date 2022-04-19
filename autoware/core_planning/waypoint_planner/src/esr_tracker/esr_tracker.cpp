#include <ros/ros.h>
#include <autoware_config_msgs/ConfigEsrTracker.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/TransformEsrObstacleList.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "esr_obstacle_info.h"

double euclideanDistance(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
	double x1 = p1.x,  x2 = p2.x;
	double y1 = p1.y,  y2 = p2.y;
	double z1 = p1.z,  z2 = p2.z;
	double xd = x1 - x2,  yd = y1 - y2,  zd = z1 - z2;
	return std::sqrt(xd*xd + yd*yd + zd*zd);
}

//停止線を設定するwaypoint情報
struct StopLineInfo
{
	int waypoint_index_; //!< 停止線を設定する経路ID
	int x_adj_; //!< 進行方向に関する微調整
	double distance_; //!<最初のwaypointから前方車両までの距離
};

class EsrTracker
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber sub_waypoints_;
	ros::Subscriber sub_config_;
	ros::Subscriber sub_esr_obstacles_;//!<esrからの障害物情報(全データ)

	ros::Publisher pub_waypoints_;
	ros::Publisher pub_front_esr_obstacle_;//!<前方障害物と判断されたesr情報
	ros::Publisher pub_marker_;

	double front_bumper_to_baselink_; //!< baselinkからフロントバンパーまでの距離
	double vehicle_width_;//!< 車両の幅
	autoware_config_msgs::ConfigEsrTracker config_;
	EsrObstacleInfo foreground_esr_; //!< 探索範囲内で見つかったesr正面オブジェクト
	std::vector<EsrObstacleInfo> esr_obstacle_info_list_; //!<mobileyeからの車両情報のリスト

	void callbackConfig(const autoware_config_msgs::ConfigEsrTracker::ConstPtr &msg)
	{
		config_ = *msg;
	}

	void callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
	{
		searchEsrFollowing(msg);

		pub_waypoints_.publish(msg);
	}

	//mobileye車両オブジェクトを追加
	void addEsrObstract(const autoware_msgs::TransformEsrObstacleList::ConstPtr &msg)
	{
		for(const autoware_msgs::TransformEsrObstacle &esr : msg->obstacles)
		{
			bool add_flag = false;
			for(EsrObstacleInfo &obs_list : esr_obstacle_info_list_)
			{
				const autoware_msgs::TransformEsrObstacle &obs = obs_list.getObs(0);
				if(obs.orig_data.id == esr.orig_data.id)//オブジェクトIDが同一なら追加
				{
					obs_list.push(esr);
					add_flag = true;
				}
			}

			if(add_flag == false)//オブジェクトIDが既存になければ追加
			{
				EsrObstacleInfo new_info;
				new_info.push(esr);
				esr_obstacle_info_list_.push_back(new_info);
			}
		}
	}

	int seq_count = 0;
	StopLineInfo searchEsrFollowing(const autoware_msgs::Lane::ConstPtr &lane)
	{
		//std::cout << "size," << esr_obstacle_info_list_.size() << std::endl;
		int hit_obs_index = -1; //判定車両ID
		int hit_waypoint_index = -1; //判定waypoint
		double x_adj = 0; //判定waypointのx微調整位置

		//経路上の正面車両を探索
		for(int waycou=1; waycou<lane->waypoints.size(); waycou++)
		{
			const autoware_msgs::Waypoint waypoint = lane->waypoints[waycou];
			const autoware_msgs::Waypoint prev_waypoint = lane->waypoints[waycou-1];
			const geometry_msgs::Point way_po = waypoint.pose.pose.position;
			const geometry_msgs::Point prev_way_po = prev_waypoint.pose.pose.position;

			for(int obscou=0; obscou<esr_obstacle_info_list_.size(); obscou++)
			{
				const EsrObstacleInfo &obs_info = esr_obstacle_info_list_[obscou];
				//geometry_msgs::Quaternion curr_qua = waypoint.pose.pose.orientation;
				autoware_msgs::TransformEsrObstacle latest_obs = obs_info.getLatestObs();
				geometry_msgs::Point obs_po = latest_obs.map_pose.position;

				//位置の分散が指定数値より大きい(該当IDの位置が安定していない)場合は処理をしない
				//geometry_msgs::Point rel_var = obs_info.getRelativePoseCov();
				//if(rel_var.x > config_.pose_variance_th || rel_var.y > config_.pose_variance_th) continue;

				//std::cout << "y," << obs_po.y << "," << prev_way_po.y << std::endl;
				Eigen::Vector3d way_diff_po(way_po.x-prev_way_po.x, way_po.y-prev_way_po.y, way_po.z-prev_way_po.z);
				Eigen::Vector3d obs_diff_po(obs_po.x-prev_way_po.x, obs_po.y-prev_way_po.y, obs_po.z-prev_way_po.z);
				Eigen::AngleAxisd way_yaw(std::atan2(way_diff_po.y(), way_diff_po.x()), Eigen::Vector3d::UnitZ());
				Eigen::Quaterniond rot_yaw(way_yaw.inverse());
				Eigen::Vector3d way_diff_po_rot_yaw = rot_yaw * way_diff_po;
				Eigen::Vector3d obs_diff_po_rot_yaw = rot_yaw * obs_diff_po;
				Eigen::AngleAxisd way_pitch(std::atan2(way_diff_po_rot_yaw.z(), way_diff_po_rot_yaw.x()), Eigen::Vector3d::UnitY());
				Eigen::Quaterniond rot_pitch(way_pitch.inverse());
				Eigen::Vector3d way_diff_po_rot = rot_pitch * way_diff_po_rot_yaw;
				Eigen::Vector3d obs_diff_po_rot = rot_pitch * obs_diff_po_rot_yaw;

				//std::cout << std::abs(obs_diff_po_rot.y()) << "<=" << config_.judg_obj_pose_y << std::endl;
				//std::cout << obs_diff_po_rot.x() << ">= 0" << std::endl;
				//std::cout << obs_diff_po_rot.x() << "<=" << way_diff_po_rot.x() << std::endl;
				if(std::abs(obs_diff_po_rot.y()) <= config_.judg_obj_pose_y
					&& obs_diff_po_rot.x() >= 0
					&& obs_diff_po_rot.x() <= way_diff_po_rot.x())
				{
					//std::cout << "id," << latest_obs.orig_data.id << std::endl;
					hit_obs_index = obscou;
					hit_waypoint_index = waycou;
					x_adj = way_diff_po_rot.x() - obs_diff_po_rot.x();
				}
			}

			if(hit_obs_index != -1) break;
		}

		//正面車両が存在した場合は停止線情報を設定
		std::cout << "hit_esr," << hit_obs_index << std::endl;
		if(hit_obs_index != -1)
		{
			double dis_sum = x_adj;
			double dis_adj = 0;
			int stop_index = 0;
			bool setflag = false;

			for(int waycou=hit_waypoint_index; waycou>=1; waycou--)
			{
				const autoware_msgs::Waypoint waypoint = lane->waypoints[waycou];
				const autoware_msgs::Waypoint prev_waypoint = lane->waypoints[waycou-1];
				const geometry_msgs::Point way_po = waypoint.pose.pose.position;
				const geometry_msgs::Point prev_way_po = prev_waypoint.pose.pose.position;
				double euc_distance = euclideanDistance(way_po, prev_way_po);
				dis_sum += euc_distance;
				if(dis_sum >= config_.min_free_running_distance && setflag == false)
				{
					dis_adj = dis_sum - config_.min_free_running_distance;
					stop_index = waycou;
					setflag = true;
					//break;
				}
			}

			foreground_esr_ = esr_obstacle_info_list_[hit_obs_index];
			StopLineInfo info;
			info.waypoint_index_ = stop_index;
			info.x_adj_ = dis_adj;
			info.distance_ = dis_sum;
			autoware_msgs::TransformEsrObstacle foreg = foreground_esr_.getLatestObs();
			foreg.distance = dis_sum;
			foreg.rel_pose_ave = foreground_esr_.getRelativePoseAve();
			foreg.map_pose_ave = foreground_esr_.getMapPoseAve();
			foreg.rel_pose_var_ave = foreground_esr_.getRelativePoseCov();
			pub_front_esr_obstacle_.publish(foreg);

			visualization_msgs::Marker marker;
			marker.header.stamp = ros::Time::now();
			marker.header.frame_id = "esr_1";
			marker.header.seq= seq_count;
			seq_count++;
			marker.id = foreg.orig_data.id;
			marker.type = 1;
			marker.action = 0;
			marker.pose = foreg.orig_data.pose.pose;
			marker.scale.x = 0.25;
			marker.scale.y = 0.25;
			marker.scale.z = 1.0;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;
			marker.lifetime = ros::Duration(0.0, 75000000);
			marker.frame_locked = false;
			marker.mesh_use_embedded_materials = false;
			pub_marker_.publish(marker);
			return info;
		}
		else
		{
			foreground_esr_.clear();
			StopLineInfo info;
			info.waypoint_index_ = -1;
			info.x_adj_ = -1;
			info.distance_ = -1;
			autoware_msgs::TransformEsrObstacle msg;
			msg.orig_data.id= UINT_MAX;
			pub_front_esr_obstacle_.publish(msg);
			return info;
		}
	}

	//esrからの障害物情報を受け取るコールバック
	void callbackEsrObstacles(const autoware_msgs::TransformEsrObstacleList::ConstPtr &msg)
	{
		addEsrObstract(msg);
	}

		//正面判定されたesr情報がconfig_.obs_delete_time秒間更新が無かった場合、その車両情報を消去する
	void callbackObsDeleteTimer(const ros::TimerEvent &e)
	{
		ros::Time ros_nowtime = ros::Time::now();//e.current_real;
		for(long i=esr_obstacle_info_list_.size()-1; i>=0; i--)
		{
			EsrObstacleInfo obs_info = esr_obstacle_info_list_[i];
			autoware_msgs::TransformEsrObstacle obs = obs_info.getLatestObs();
			ros::Duration ros_time_diff = ros_nowtime - obs.header.stamp;
			double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
			if(time_diff > config_.obs_delete_time) esr_obstacle_info_list_.erase(esr_obstacle_info_list_.begin() + i);
		}
	}
public:
	EsrTracker(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		front_bumper_to_baselink_ = nh_.param<double>("/vehicle_info/front_bumper_to_baselink", 4.55);
		vehicle_width_ = nh_.param<double>("/vehicle_info/vehicle_width", 2.3);

		sub_config_ = nh_.subscribe<autoware_config_msgs::ConfigEsrTracker>(
			"/config/esr_tracker", 10, &EsrTracker::callbackConfig, this);
		sub_waypoints_ = nh_.subscribe<autoware_msgs::Lane>(
			"/position_adjustment_waypoints", 10, &EsrTracker::callbackWaypoints, this);
		sub_esr_obstacles_ = nh_.subscribe<autoware_msgs::TransformEsrObstacleList>(
			"/transform_esr_obstacles", 10, &EsrTracker::callbackEsrObstacles, this);

		pub_waypoints_ = nh_.advertise<autoware_msgs::Lane>("/esr_tracking_waypoints", 10);
		pub_front_esr_obstacle_ = nh_.advertise<autoware_msgs::TransformEsrObstacle>("/esr_tracker/front_esr", 10);
		pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/esr_tracker/front_marker", 10);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "esr_tracker");
	ros::NodeHandle nh, pnh("~");

	EsrTracker tracker(nh, pnh);
	ros::spin();
	return 0;
}