#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <autoware_msgs/LaneArray.h>

// headers in PCL
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

class MapWaypointsSelect
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Publisher pub_new_points_map_;
	ros::Subscriber sub_global_lanes_;
	ros::Subscriber sub_points_map_;

	std::string outpath_;//PCDファイルの出力ファイル名
	double distance_th_;//waypointとmap点がこの距離以内のmap点を使用する。
	int start_id_;//map点選別に使用するwaypointのstartID
	int end_id_;//map点選別に使用するwaypointのendID
	std::vector<geometry_msgs::Point> waypoints_;//subscribeしたwaypoint一覧
	sensor_msgs::PointCloud2 points_map_;//subscribeしたマップ情報

	double euclidDistance(double x1, double y1, double z1, double x2, double y2, double z2)
	{
		double x = x1-x2, y = y1-y2, z = z1-z2;
		return std::sqrt(x*x + y*y + z*z);
	}

	//pointcloudの[field]情報からbyteデータ列[dp]から数値を取得/
	//[bigendian]はエンディアン情報
	double valueConversion(const uint8_t* dp, const sensor_msgs::PointField &field, const uint8_t bigendian)
	{
		switch(field.datatype)
		{
		case sensor_msgs::PointField::FLOAT32:
			{
				uint8_t bytes[4];
				float *data = reinterpret_cast<float*>(bytes);
				if(bigendian == 0)
					for(int i=0; i<4; i++) bytes[i] = dp[i];
				else
					for(int i=0; i<4; i++) bytes[i] = dp[4-i-1];
				return *data;
			}
		case sensor_msgs::PointField::FLOAT64:
			{
				uint8_t bytes[8];
				double *data = reinterpret_cast<double*>(bytes);
				if(bigendian == 0)
					for(int i=0; i<8; i++) bytes[i] = dp[i];
				else
					for(int i=0; i<8; i++) bytes[i] = dp[8-i-1];
				return *data;
			}
		case sensor_msgs::PointField::INT8:
			{
				return static_cast<int8_t>(dp[0]);
			}
		case sensor_msgs::PointField::UINT8:
			{
				return dp[0];
			}
		case sensor_msgs::PointField::INT16:
			{
				uint8_t bytes[2];
				int16_t *data = reinterpret_cast<int16_t*>(bytes);
				if(bigendian == 0)
					for(int i=0; i<2; i++) bytes[i] = dp[i];
				else
					for(int i=0; i<2; i++) bytes[i] = dp[2-i-1];
				return *data;
			}
		case sensor_msgs::PointField::UINT16:
			{
				uint8_t bytes[2];
				uint16_t *data = reinterpret_cast<uint16_t*>(bytes);
				if(bigendian == 0)
					for(int i=0; i<2; i++) bytes[i] = dp[i];
				else
					for(int i=0; i<2; i++) bytes[i] = dp[2-i-1];
				return *data;
			}
		case sensor_msgs::PointField::INT32:
			{
				uint8_t bytes[4];
				int32_t *data = reinterpret_cast<int32_t*>(bytes);
				if(bigendian == 0)
					for(int i=0; i<4; i++) bytes[i] = dp[i];
				else
					for(int i=0; i<4; i++) bytes[i] = dp[4-i-1];
				return *data;
			}
		case sensor_msgs::PointField::UINT32:
			{
				uint8_t bytes[4];
				uint32_t *data = reinterpret_cast<uint32_t*>(bytes);
				if(bigendian == 0)
					for(int i=0; i<4; i++) bytes[i] = dp[i];
				else
					for(int i=0; i<4; i++) bytes[i] = dp[4-i-1];
				return *data;
			}
		default:
			{
				return DBL_MAX;
			}
		}
	}

	//global_lanes_のwaypoint周辺に存在するmap点群を探索してpublish
	void selectPointMap()
	{
		if(waypoints_.size() == 0 || points_map_.data.size() == 0)
		{
			return;
		}

		std::cout << "height," << points_map_.height << std::endl;
		std::cout << "width," << points_map_.width << std::endl;
		std::cout << "bigendian," << +points_map_.is_bigendian << std::endl;
		std::cout << "point_step," << points_map_.point_step << std::endl;
		std::cout << "row_step," << points_map_.row_step << std::endl;
		std::cout << "is_dense," << +points_map_.is_dense << std::endl;
		std::cout << "fields" << std::endl;

		bool existenceX=false, existenceY=false, existenceZ=false;//filedに座標情報があるかのチェック
		for(const sensor_msgs::PointField &field : points_map_.fields)
		{
			std::cout << "  name," << field.name << std::endl;
			std::cout << "  offset," << field.offset << std::endl;
			std::cout << "  datatyep," << +field.datatype << std::endl;
			std::cout << "  count," << field.count << std::endl;
			std::cout << "-----------" << std::endl;

			if(field.name == "x") existenceX = true;
			if(field.name == "y") existenceY = true;
			if(field.name == "z") existenceZ = true;
			/*for(const autoware_msgs::Lane &lane : global_lanes_.lanes)
			{
				for(int waycou=0; waycou<lane.waypoints.size(); waycou++)
				{
					const geometry_msgs::Point &po = lane.waypoints[waycou].pose.pose.position;

				}
			}*/
		}
		if(!existenceX)
		{
			std::cout << "error : not x field" << std::endl;
			return;
		}
		if(!existenceY)
		{
			std::cout << "error : not y field" << std::endl;
			return;
		}
		if(!existenceZ)
		{
			std::cout << "error : not z field" << std::endl;
			return;
		}

		//データ整合性チェック
		size_t point_count = (size_t)points_map_.height * (size_t)points_map_.width;
		size_t data_size = point_count * points_map_.point_step;
		std::cout << "data_size(math)," << data_size << std::endl;
		std::cout << "data_size(point_map.data.size)," << points_map_.data.size() << std::endl;
		if(data_size != points_map_.data.size())
		{
			std::cout << "error : data size mismatch" << std::endl;
			return;
		}

		sensor_msgs::PointCloud2 new_map;
		new_map.header = points_map_.header;
		new_map.header.stamp = ros::Time::now();
		new_map.height = 1;
		new_map.width = 0;
		new_map.fields = points_map_.fields;
		new_map.is_bigendian = points_map_.is_bigendian;
		new_map.point_step = points_map_.point_step;
		new_map.row_step = 0;
		new_map.is_dense = 1;

		for(size_t pcou=0; pcou<point_count; pcou++)
		{
			const uint8_t* dataP = &points_map_.data[pcou * points_map_.point_step];

			//--------xyz座標の読み込み---------
			double mx=DBL_MAX, my=DBL_MAX, mz=DBL_MAX;
			for(int fcou=0; fcou<points_map_.fields.size(); fcou++)
			{
				sensor_msgs::PointField &field = points_map_.fields[fcou];
				const uint8_t* dp = &dataP[field.offset];
				double coord_val  = valueConversion(dp, field, points_map_.is_bigendian);
				if(coord_val == DBL_MAX)
				{
					std::cout << "error : field type mismatch" << std::endl;
					return;	
				}
				if(field.name == "x")      mx = coord_val;
				else if(field.name == "y") my = coord_val;
				else if(field.name == "z") mz = coord_val;
			}
			if(mx == DBL_MAX)
			{
				std::cout << "error : not x coordinate" << std::endl;
				return;
			}
			if(my == DBL_MAX)
			{
				std::cout << "error : not y coordinate" << std::endl;
				return;
			}
			if(mz == DBL_MAX)
			{
				std::cout << "error : not z coordinate" << std::endl;
				return;
			}

			//--------waypoint座標に近いpoints mapを取得---------
			bool waypoint_within_range = false;
			for(const geometry_msgs::Point &po : waypoints_)
			{
				if(euclidDistance(po.x, po.y, po.z, mx, my, mz) <= distance_th_)
				{
					waypoint_within_range = true;
					break;
				}
			}
			if(waypoint_within_range == true)
			{
				for(uint32_t i=0; i<points_map_.point_step; i++)
				new_map.data.push_back(dataP[i]);
				new_map.width++;
				new_map.row_step += points_map_.point_step;
			}
		}

		std::cout << "convert end" << std::endl;
		pcl::io::savePCDFile(outpath_, new_map,
			Eigen::DenseBase<Eigen::Vector4f>::Zero(), Eigen::QuaternionBase<Eigen::Quaternionf>::Identity(), true);
		pub_new_points_map_.publish(new_map);
	}

	void callbackGlobalLanes(const autoware_msgs::LaneArray::ConstPtr &msg)
	{
		waypoints_.clear();
		std::cout << "read lanes" << std::endl;
		for(const autoware_msgs::Lane &lane : msg->lanes)
		{
			for(const autoware_msgs::Waypoint waypoint : lane.waypoints)
			{
				//if(waypoint.waypoint_param.id >= (uint32_t)start_id_ && waypoint.waypoint_param.id <= (uint32_t)end_id_)
					waypoints_.push_back(waypoint.pose.pose.position);
			}
		}
		selectPointMap();
	}

	void callbackPointsMap(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		std::cout << "read map" << std::endl;
		points_map_ = static_cast<sensor_msgs::PointCloud2>(*msg);
		selectPointMap();
	}
public:
	MapWaypointsSelect(const ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		outpath_ = nh_.param<std::string>("outpath", "/tmp/pcd_data.pcd");
		distance_th_ = nh_.param<double>("distance_th", 100.0);
		start_id_ = nh_.param<int>("start_id", 0);
		end_id_ = nh_.param<int>("end_id", 0);

		pub_new_points_map_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_map_wayselect", 1, true);
		sub_global_lanes_ = nh_.subscribe<autoware_msgs::LaneArray>(
			"/lane_waypoints_array", 1, &MapWaypointsSelect::callbackGlobalLanes, this);
		sub_points_map_ = nh_.subscribe<sensor_msgs::PointCloud2>(
			"/points_map", 1, &MapWaypointsSelect::callbackPointsMap, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_waypoints_select");
	ros::NodeHandle nh, pnh("~");

	MapWaypointsSelect mws(nh, pnh);
	ros::spin();
	return 0;
}