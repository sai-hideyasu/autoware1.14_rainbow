#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/AdjustXY.h>
#include <autoware_config_msgs/ConfigVoxelGridFilter.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/TrafficLight.h>
//#include <autoware_config_msgs/ConfigLocalizerSwitchFusion.h>
//#include <autoware_config_msgs/ConfigLookAhead.h>
#include <std_msgs/Float64.h>
#include <jsk_rviz_plugins/OverlayText.h>

class WaypointMaker
{
private:
	ros::NodeHandle nh_, private_nh_;

	ros::Subscriber sub_local_waypoint_;
	ros::Publisher pub_waypoint_param_, pub_adjust_xy_, pub_voxelGridFilter_, pub_roi_signal_, pub_light_color_;
	//ros::Publisher pub_look_ahead_;
	ros::Publisher pub_look_ahead_ratio_magn_, pub_waypoint_id_;

	int prev_light_color_;

	void callback_local_waypoints(const autoware_msgs::Lane& msg)
	{
		if(msg.waypoints.size() < 2)
		{
			std_msgs::String waypoint_id_str;
			waypoint_id_str.data = "-1";
			pub_waypoint_id_.publish(waypoint_id_str);
			return;
		}

		autoware_msgs::WaypointParam param = msg.waypoints[1].waypoint_param;
		pub_waypoint_param_.publish(param);

		if(param.feat_proj_x > -10000 && param.feat_proj_y > -10000)
		{
			autoware_msgs::AdjustXY ad_xy;
			ad_xy.header.frame_id = "";
			ad_xy.header.stamp = msg.header.stamp;
			ad_xy.header.seq = msg.header.seq;
			ad_xy.x = param.feat_proj_x;
			ad_xy.y = param.feat_proj_y;
			pub_adjust_xy_.publish(ad_xy);
		}

		if(param.vgf_leafsize > 0)
		{
			autoware_config_msgs::ConfigVoxelGridFilter cvgf;
			cvgf.voxel_leaf_size = param.vgf_leafsize;
			cvgf.measurement_range = param.vgf_measurement_range;
			pub_voxelGridFilter_.publish(cvgf);
		}

		/*if(param.signals.size() == 3)
		{
			autoware_msgs::Signals sig_msg;
			sig_msg.header.frame_id = "";
			sig_msg.header.stamp = ros::Time::now();
			sig_msg.header.seq = 0;
			for(int i=0;i<3;i++)
			{
				autoware_msgs::ExtractedPosition ep;
				ep.signalId = param.signals[i].signalId;
				ep.u = param.signals[i].u;
				ep.v = param.signals[i].v;
				ep.radius = param.signals[i].radius;
				ep.x = param.signals[i].x;
				ep.y = param.signals[i].y;
				ep.z = param.signals[i].z;
				ep.hang = param.signals[i].hang;
				ep.type = param.signals[i].type;
				ep.linkId = param.signals[i].linkId;
				ep.plId = param.signals[i].plId;
				sig_msg.Signals.push_back(ep);
			}
			pub_roi_signal_.publish(sig_msg);
		}*/

		/*if(param.fusion_select >= 0)
		{
			std_msgs::Int32 msg;
			msg.data = param.fusion_select;
			pub_fusion_select_.publish(msg);
		}*/

		/*if(param.lookahead_ratio > 0 && param.minimum_lookahead_distance > 0)
		{
			autoware_config_msgs::ConfigLookAhead msg;
			msg.header.frame_id = "";
			msg.header.stamp = ros::Time::now();
			msg.header.seq = 0;
			msg.lookahead_ratio = param.lookahead_ratio;
			msg.minimum_lookahead_distance = param.minimum_lookahead_distance;
			pub_look_ahead_.publish(msg);
		}*/

		if(param.lookahead_ratio_magn > 0)
		{
			std_msgs::Float64 msg;
			msg.data = param.lookahead_ratio_magn;
			pub_look_ahead_ratio_magn_.publish(msg);
		}

		if(param.pub_light_color != -1)
		{
			if(prev_light_color_ == -1 || prev_light_color_ != param.pub_light_color)
			{
				autoware_msgs::TrafficLight tl;
				tl.header.stamp = msg.header.stamp;
				tl.traffic_light = param.pub_light_color;
				pub_light_color_.publish(tl);
			}
			prev_light_color_ = param.pub_light_color;
		}
		else prev_light_color_ = -1;

		jsk_rviz_plugins::OverlayText waypoint_id_str;
		std::stringstream ss;
		ss << "ID:" << msg.waypoints[1].waypoint_param.id;
		waypoint_id_str.text = ss.str();
		//waypoint_id_str.text = std::to_string(msg.waypoints[1].waypoint_param.id);
		pub_waypoint_id_.publish(waypoint_id_str);
	}
	
public:
	WaypointMaker(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: prev_light_color_(-1)
	{
		nh_ = nh;  private_nh_ = p_nh;

		pub_adjust_xy_ = nh.advertise<autoware_msgs::AdjustXY>("/config/adjust_xy",10);
		pub_voxelGridFilter_ = nh.advertise<autoware_config_msgs::ConfigVoxelGridFilter>("/config/voxel_grid_filter",10);
		pub_roi_signal_ = nh.advertise<autoware_msgs::Signals>("/loader_roi_signal",10);
		pub_waypoint_param_ = nh_.advertise<autoware_msgs::WaypointParam>("/waypoint_param", 1);
		//pub_fusion_select_ = nh_.advertise<autoware_config_msgs::ConfigLocalizerSwitchFusion>("/config/localizer_switch", 1);
		//pub_look_ahead_ = nh_.advertise<autoware_config_msgs::ConfigLookAhead>("/config/look_ahead", 1);
		pub_look_ahead_ratio_magn_ = nh_.advertise<std_msgs::Float64>("/lookahead_ratio_magn", 1);
		pub_waypoint_id_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("/first_waypoint_id_str", 1);
		pub_light_color_ = nh_.advertise<autoware_msgs::TrafficLight>("/light_color", 1);
		sub_local_waypoint_ = nh_.subscribe("/final_waypoints", 10, &WaypointMaker::callback_local_waypoints, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_param_sender");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	WaypointMaker waypoint_maker(nh, private_nh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
