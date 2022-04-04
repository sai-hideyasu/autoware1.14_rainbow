#include <ros/ros.h>
#include <fstream>
#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <autoware_config_msgs/ConfigWaypointExtension.h>

std::vector<std::string> split(const std::string &string, const char sep)
{
	std::vector<std::string> str_vec_ptr;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, sep))
		str_vec_ptr.push_back(token);

	return str_vec_ptr;
}

class WaypointExtension
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Subscriber sub_config_;

	autoware_config_msgs::ConfigWaypointExtension config_;
	std::vector<std::string> columns_list_;
	std::vector<std::unordered_map<std::string, std::string>> waypoint_list_;

	void callbackConfig(const autoware_config_msgs::ConfigWaypointExtension::ConstPtr &msg)
	{
		std::unordered_map<std::string, std::string> end_map = waypoint_list_[waypoint_list_.size()-1];
		const double end_x   = atof(end_map["x"].c_str());
		const double end_y   = atof(end_map["y"].c_str());
		const double end_z   = atof(end_map["z"].c_str());
		const double end_yaw = atof(end_map["yaw"].c_str());

		for(unsigned int i=1; i<=msg->loop_count; i++)
		{
			const Eigen::Vector3d interval_pos = Eigen::Vector3d(i * msg->waypoint_interval, 0, 0);
			const Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(end_yaw, Eigen::Vector3d::UnitZ()));
			const Eigen::Vector3d pos_rot = qua * interval_pos;
			const Eigen::Vector3d pos_move = pos_rot + Eigen::Vector3d(end_x, end_y, end_z);

			std::unordered_map<std::string, std::string> map;
			for(const std::string column : columns_list_)
			{
				map[column] = end_map[column];
			}
			std::stringstream ssx, ssy, ssz, ssyaw;
			ssx << std::fixed << std::setprecision(4) << pos_move.x();
			ssy << std::fixed << std::setprecision(4) << pos_move.y();
			ssz << std::fixed << std::setprecision(4) << pos_move.z();
			ssyaw << std::fixed << std::setprecision(4) << end_yaw;
			map["x"] = ssx.str();
			map["y"] = ssy.str();
			map["z"] = ssz.str();
			map["yaw"] = ssyaw.str();
			waypoint_list_.push_back(map);
		}

		std::ofstream ofs("/home/sit/new_waypoints.csv", std::ios_base::out);
		for(int i=0; i<columns_list_.size(); i++)
		{
			ofs << columns_list_[i];
			if(i != columns_list_.size() -1) ofs << ",";
		}
		ofs << std::endl;
		for(int i=0; i<waypoint_list_.size(); i++)
		{
			std::unordered_map<std::string, std::string> map = waypoint_list_[i];
			//for(const std::string column : columns_list_)
			for(int j=0; j<columns_list_.size(); j++)
			{
				const std::string column = columns_list_[j];
				ofs << map[column];
				if(j != columns_list_.size() -1) ofs << ",";
			}
			ofs << std::endl;
		}
		ofs.close();
	}
public:
	WaypointExtension(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
    {
		sub_config_ = nh_.subscribe("/config/waypoint_extension", 1, &WaypointExtension::callbackConfig, this);

		std::string waypoint_file = pnh_.param<std::string>("waypoint_file", "");
		std::ifstream ifs(waypoint_file, std::ios_base::in);

		std::string columns_str;
		std::getline(ifs, columns_str);
		columns_list_ = split(columns_str, ',');
		while(!ifs.eof())
		{
			std::unordered_map<std::string, std::string> map;

			std::string str;
			std::getline(ifs, str);
			std::vector<std::string> list = split(str, ',');
			if(columns_list_.size() != list.size()) continue;
			for (size_t i = 0; i < columns_list_.size(); i++)
			{
				//std::cout << "aaa," << contents.at(i) << "," << columns.at(i) << "," << std::endl;
				map[columns_list_.at(i)] = list.at(i);
				//std::cout << map["x"] << std::endl;
			}
			waypoint_list_.push_back(map);
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_extension");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	WaypointExtension we(nh, pnh);
	ros::spin();
	return 0;
}
