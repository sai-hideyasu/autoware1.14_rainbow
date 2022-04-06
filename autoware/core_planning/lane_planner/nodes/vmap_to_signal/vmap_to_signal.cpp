#include <ros/ros.h>
#include <fstream>
#include <vector_map/vector_map.h>
#include <lane_planner/lane_planner_vmap.hpp>
#include <autoware_msgs/LaneArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct StopLineCoordinate
{
	geometry_msgs::Point bp_;
	geometry_msgs::Point fp_;
};

class VmapToSignal
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Publisher pub_add_signal_lane_;//信号情報を付与した経路群をpublish

	ros::Subscriber sub_vmap_stopline_;//VectorMapの停止線情報
	ros::Subscriber sub_vmap_line_;//VectorMapの線情報
	ros::Subscriber sub_vmap_point_;//VectorMapの座標情報
	ros::Subscriber sub_lane_array_;//読み込まれた経路群情報

	lane_planner::vmap::VectorMap all_vmap_;
	autoware_msgs::LaneArray lane_array_;

	void callbaclVmapStopline(const vector_map_msgs::StopLineArray::ConstPtr& msg)
	{
		all_vmap_.stoplines = msg->data;
		puslishApplySignalLaneArray();
	}

	void callbaclVmapline(const vector_map_msgs::LineArray::ConstPtr& msg)
	{
		all_vmap_.lines = msg->data;
		puslishApplySignalLaneArray();
	}

	void callbaclVmapPoint(const vector_map_msgs::PointArray::ConstPtr& msg)
	{
		all_vmap_.points = msg->data;
		puslishApplySignalLaneArray();
	}

	//停止線と交差しているwaypointに信号情報を付与
	autoware_msgs::Lane applyStopline(const autoware_msgs::Lane& lane)
	{
		std::vector<StopLineCoordinate> stop_line_coord = getStopLineCoordinate();
		//std::cout << "coord size," << stop_line_coord.size() << std::endl;

		autoware_msgs::Lane new_lane = lane;
		std::vector<autoware_msgs::Waypoint> &waypoints = new_lane.waypoints;

		for(int waycou=0; waycou<waypoints.size()-1; waycou++)
		{
			geometry_msgs::Point wp1 = waypoints[waycou].pose.pose.position;
			geometry_msgs::Point wp2 = waypoints[waycou+1].pose.pose.position;

			for(const StopLineCoordinate sld : stop_line_coord)
			{
				geometry_msgs::Point vp1 = sld.bp_;
				geometry_msgs::Point vp2 = sld.fp_;

				Eigen::Vector3d wp2m(wp2.x-wp1.x, wp2.y-wp1.y, wp2.z-wp1.z);
				Eigen::Vector3d vp1m(vp1.x-wp1.x, vp1.y-wp1.y, vp1.z-wp1.z);
				Eigen::Vector3d vp2m(vp2.x-wp1.x, vp2.y-wp1.y, vp2.z-wp1.z);
				Eigen::Quaterniond rot(Eigen::AngleAxisd(-atan2(wp2.y-wp1.y, wp2.x-wp1.x), Eigen::Vector3d::UnitZ()));
				Eigen::Vector3d wp2m_rot = rot * wp2m;
				Eigen::Vector3d vp1m_rot = rot * vp1m;
				Eigen::Vector3d vp2m_rot = rot * vp2m;

				if(vp2m_rot.x() == vp1m_rot.x()) continue;//傾き計算不可
				if(vp2m_rot.y() * vp1m_rot.y() > 0) continue;//x軸と交わっていない

				//現在のwaypoint座標を中心としたVectorMapLineとの交点を求める
				double slop = (vp2m_rot.y() - vp1m_rot.y()) / (vp2m_rot.x() - vp1m_rot.x());
				double intercept = vp2m_rot.y() - slop * vp2m_rot.x();
				double intersection = -intercept/slop;
				if(intersection >= 0 && wp2m_rot.x() >= intersection)
				{
					std::cout << waypoints[waycou].waypoint_param.id << "," << vp2m_rot.x() * vp1m_rot.x() << "," << wp1.x << "," << wp1.y << "," << wp2m_rot.x() << "," << intersection << std::endl;
					waypoints[waycou].waypoint_param.signal_stop_line = 1;
					waypoints[waycou].waypoint_param.stop_line_adjustment = intersection;
					waypoints[waycou].waypoint_param.temporary_fixed_velocity_kmh = 0;
				}
			}
		}

		return new_lane;
	}

	//各レーンにVector Mapを参照した信号情報を付与
	void puslishApplySignalLaneArray()
	{
		if(!vmapCheck() && lane_array_.lanes.size() == 0)
		{
			pub_add_signal_lane_.publish(lane_array_);
			return;
		}

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = "map";

		autoware_msgs::LaneArray signal_lane_array;
		for (size_t lane_cou=0; lane_cou<lane_array_.lanes.size(); lane_cou++)
		{
			autoware_msgs::Lane lane = create_new_lane(lane_array_.lanes[lane_cou], header);
			lane_planner::vmap::VectorMap coarse_vmap = lane_planner::vmap::create_coarse_vmap_from_lane(lane);
			if (coarse_vmap.points.size() < 2)
			{
				signal_lane_array.lanes.push_back(lane);
				continue;
			}

			lane_planner::vmap::VectorMap fine_vmap = lane_planner::vmap::create_fine_vmap(
				all_vmap_, lane_planner::vmap::LNO_ALL, coarse_vmap, 1.0, 10000);
			if (fine_vmap.points.size() < 2 || !is_fine_vmap(fine_vmap, lane))
				signal_lane_array.lanes.push_back(applyStopline(lane));
			else signal_lane_array.lanes.push_back(lane);
		}

		pub_add_signal_lane_.publish(signal_lane_array);

		std::ofstream ofs("/tmp/signal_list.csv");
		for(int lane_cou=0; lane_cou<signal_lane_array.lanes.size(); lane_cou++)
		{
			ofs << "lane1" << std::endl;
			ofs << "id,signal_stop_line,stop_line_adjustment,temporary_fixed_velocity" << std::endl;

			const autoware_msgs::Lane &lane = signal_lane_array.lanes[lane_cou];
			for(int waycou=0; waycou<lane.waypoints.size(); waycou++)
			{
				const autoware_msgs::WaypointParam &param = lane.waypoints[waycou].waypoint_param;
				ofs << param.id << "," << +param.signal_stop_line << "," << param.stop_line_adjustment << "," << param.temporary_fixed_velocity_kmh << std::endl;
			}
			ofs << std::endl;
		}
		ofs.close();
	}

	void callbackLaneArray(const autoware_msgs::LaneArray::ConstPtr& msg)
	{
		lane_array_ = *msg;
		puslishApplySignalLaneArray();
	}

	bool vmapCheck()
	{
		//std::cout << std::boolalpha << !all_vmap_.stoplines.empty() << "," << !all_vmap_.lines.empty() << "," << !all_vmap_.points.empty() << std::endl;

		//if (all_vmap_.points.empty() || all_vmap_.lanes.empty() || all_vmap_.nodes.empty() || all_vmap_.stoplines.empty() ||
		//	all_vmap_.dtlanes.empty()) return false;
		if (all_vmap_.stoplines.empty() || all_vmap_.lines.empty() || all_vmap_.points.empty()) return false;
		return true;
	}

	bool is_fine_vmap(const lane_planner::vmap::VectorMap& fine_vmap, const autoware_msgs::Lane& lane)
	{
		if (fine_vmap.points.size() != lane.waypoints.size())
			return false;

		for (size_t i = 0; i < fine_vmap.points.size(); ++i)
		{
			vector_map::Point point = lane_planner::vmap::create_vector_map_point(lane.waypoints[i].pose.pose.position);
			double distance = hypot(fine_vmap.points[i].bx - point.bx, fine_vmap.points[i].ly - point.ly);
			if (distance > 0.1)
			return false;
		}

		return true;
	}

	autoware_msgs::Lane create_new_lane(const autoware_msgs::Lane& lane, const std_msgs::Header& header)
	{
		autoware_msgs::Lane l = lane;
		l.header = header;

		for (autoware_msgs::Waypoint& w : l.waypoints)
		{
			w.pose.header = header;
			w.twist.header = header;
		}

		return l;
	}

	std::vector<StopLineCoordinate> getStopLineCoordinate()
	{
		std::vector<StopLineCoordinate> ret;

		for(const vector_map_msgs::StopLine stopline : all_vmap_.stoplines)
		{
			for(const vector_map_msgs::Line line : all_vmap_.lines)
			{
				if(stopline.lid != line.lid) continue;

				StopLineCoordinate coord;
				coord.bp_.x = coord.fp_.x = DBL_MIN;
				for(const vector_map_msgs::Point point : all_vmap_.points)
				{
					if(line.bpid == point.pid)
					{
						coord.bp_.x = point.ly;
						coord.bp_.y = point.bx;
						coord.bp_.z = point.h;
					}
					if(line.fpid == point.pid)
					{
						coord.fp_.x = point.ly;
						coord.fp_.y = point.bx;
						coord.fp_.z = point.h;
					}
				}

				if(coord.bp_.x != DBL_MIN && coord.fp_.x != DBL_MIN)
				{
					ret.push_back(coord);
				}
			}
		}

		return ret;
	}
public:
	VmapToSignal(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		pub_add_signal_lane_ = nh_.advertise<autoware_msgs::LaneArray>("/assignment_signal_waypoints_array", 1, true);

		sub_vmap_stopline_ = nh_.subscribe<vector_map_msgs::StopLineArray>
			("/vector_map_info/stop_line", 1, &VmapToSignal::callbaclVmapStopline, this);
		sub_vmap_line_ = nh_.subscribe<vector_map_msgs::LineArray>
			("/vector_map_info/line", 1, &VmapToSignal::callbaclVmapline, this);
		sub_vmap_point_ = nh_.subscribe<vector_map_msgs::PointArray>
			("/vector_map_info/point", 1, &VmapToSignal::callbaclVmapPoint, this);
		sub_lane_array_ = nh_.subscribe<autoware_msgs::LaneArray>
			("/lane_waypoints_array", 1, &VmapToSignal::callbackLaneArray, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vmap_to_signal");
	ros::NodeHandle nh, pnh("~");

	VmapToSignal vts(nh, pnh);
	ros::spin();
	return 0;
}