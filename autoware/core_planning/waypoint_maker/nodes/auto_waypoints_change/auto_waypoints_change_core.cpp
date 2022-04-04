#include "auto_waypoints_change.h"

namespace auto_waypoints_change
{
std::vector<std::string> split(const std::string &string, const char sep)
{
	std::vector<std::string> str_vec_ptr;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, sep))
		str_vec_ptr.push_back(token);

	return str_vec_ptr;
}

//waypoint間のdistance計算
double waypointDT(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
	double x = p1.x - p2.x,   y = p1.y - p2.y,   z = p1.z - p2.z;
	return sqrt(x*x + y*y);// + z*z);
}

//ファイル存在チェック
int existFile(const char* path)
{
	//pathが存在するか
	struct stat st;
	if (stat(path, &st) != 0) {
		return 0;
	}

	// ファイルかどうか
	return (st.st_mode & S_IFMT) == S_IFREG;
}

//global waypoint生成ノードをストップ
void AutoWaypointsChange::killWaypointsNode()
{
	std_msgs::Empty emp_msg;
	pub_astar_avoid_end_.publish(emp_msg);
	system("rosnode kill /waypoint_loader_show_id &");
	system("rosnode kill /waypoint_replanner &");
	system("rosnode kill /waypoint_marker_publisher_show_id &");
	system("rosnode kill /points_map_loader &");
	usleep(1000);
}

//launch引数で指定された経路用launchファイルを読み込み
void AutoWaypointsChange::runWaypointsNode(std::string launch)
{
	std::string launch_str = ros::package::getPath("waypoint_maker") + "/launch/" + launch + ".launch";
	if(existFile(launch_str.c_str()) != false)
	{
		std::string cmd = "roslaunch waypoint_maker " + launch + ".launch &";
		system(cmd.c_str());
		usleep(3000);
		system("roslaunch waypoint_planner astar_avoid.launch &");
		route_change_time_ = ros::Time::now();
	}
}

void AutoWaypointsChange::changeRoute()
{
	ros::Duration ros_time_diff = ros::Time::now() - route_change_time_;
	double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
	if(time_diff > ROUTE_LOAD_WAIT_TIME_)
	{
		killWaypointsNode();
		runWaypointsNode(load_list_[loop_count_][lane_count_]);
	}
}

void AutoWaypointsChange::changeRouteIncrement(const bool increment)
{
	ros::Duration ros_time_diff = ros::Time::now() - route_change_time_;
	double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
	if(time_diff > ROUTE_LOAD_WAIT_TIME_)
	{
		if(increment == true) loopIncrement();
		else loopDecrement();
		killWaypointsNode();
		runWaypointsNode(load_list_[loop_count_][lane_count_]);
	}
}

void AutoWaypointsChange::changeRouteLoopNext(const bool loop_move)
{
	ros::Duration ros_time_diff = ros::Time::now() - route_change_time_;
	double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
	if(time_diff > ROUTE_LOAD_WAIT_TIME_)
	{
		if(loop_move == true) loopNext();
		else loopBack();
		killWaypointsNode();
		runWaypointsNode(load_list_[loop_count_][lane_count_]);
	}
}

void AutoWaypointsChange::callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	current_pose_ = *msg;
}

void AutoWaypointsChange::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	current_velocity_ = *msg;
}

void AutoWaypointsChange::callbackCan502(const autoware_can_msgs::MicroBusCan502::ConstPtr &msg)
{
	can502_ = *msg;
}

void AutoWaypointsChange::callbackCan503(const autoware_can_msgs::MicroBusCan503::ConstPtr &msg)
{
	can503_ = *msg;
}

void AutoWaypointsChange::callbackRunFlag(const std_msgs::Bool::ConstPtr &msg)
{
	run_flag_ = msg->data;
}

void AutoWaypointsChange::callbackRouteList(const autoware_msgs::AutoRouteList::ConstPtr &msg)
{
	load_list_.clear();
	loop_count_ = lane_count_ = 0;
	int current_loop_num = 0;
	std::vector<std::string> list;
	for(int i=0; i<msg->list.size(); i++)
	{
		std::vector<std::string> strs = split(msg->list[i],'-');
		if(strs.size() != 2)
		{
			std::cout << "1要素の記述は[周回番号]-[launchファイル名(拡張子なし)]です" << std::endl;
			return;
		}
		int loop_num = atoi(strs[0].c_str());
		if(loop_num == 0)
		{
			std::cout << "[周回番号]が0もしくは数字以外です" << std::endl;
			return;
		}
		if(current_loop_num + 1 == loop_num)
		{
			if(list.size() != 0) load_list_.push_back(list);
			list.clear();
			current_loop_num++;
			list.push_back(strs[1]);
		}
		else if(current_loop_num == loop_num)
		{
			list.push_back(strs[1]);
		}
		else
		{
			std::cout << "[周回番号]が1から始まる連番ではありません" << std::endl;
			return;
		}
		std::string launch_file_name = strs[1];
	}

	if(list.size() != 0) load_list_.push_back(list);

	if(load_list_.size() != 0)
	{
		//killWaypointsNode();
		//runWaypointsNode(load_list_[loop_count_][lane_count_]);
		changeRoute();
	}
	/*for(int i=0;i<load_list_.size();i++)
	{
		std::vector<std::string> a = load_list_[i];
		for(int j=0;j<a.size();j++)
			std::cout << a[j] << ",";
		std::cout << std::endl;
	}*/
}

void AutoWaypointsChange::loopCountPublish()
{
	autoware_msgs::AutoRouteLoopCount msg;
	msg.header.stamp = ros::Time::now();
	msg.loop = loop_count_+1;
	msg.lane = lane_count_+1;
	pub_auto_route_loop_count_.publish(msg);
}

void AutoWaypointsChange::loopIncrement()
{
	lane_count_++;
	if(lane_count_ >= load_list_[loop_count_].size())
	{
		lane_count_ = 0;
		loop_count_++;
	}
	if(loop_count_ >= load_list_.size())
	{
		lane_count_ = 0;
		loop_count_ = 0;
	}
	loopCountPublish();
}

void AutoWaypointsChange::loopDecrement()
{
	lane_count_--;
	if(lane_count_ < 0)
	{
		loop_count_--;
		if(loop_count_ < 0) loop_count_ = load_list_.size()-1;
		lane_count_ = load_list_[loop_count_].size()-1;
	}
	loopCountPublish();
}

void AutoWaypointsChange::loopNext()
{
	lane_count_ = 0;
	loop_count_++;
	if(loop_count_ >= load_list_.size()) loop_count_ = 0;
	loopCountPublish();
}

void AutoWaypointsChange::loopBack()
{
	lane_count_ = 0;
	loop_count_--;
	if(loop_count_ < 0) loop_count_ = load_list_.size() - 1;
	loopCountPublish();
}

void AutoWaypointsChange::callbackIncrement(const std_msgs::Bool::ConstPtr &msg)
{
	/*if(msg->data == true) loopIncrement();
	else loopDecrement();
	killWaypointsNode();
	runWaypointsNode(load_list_[loop_count_][lane_count_]);*/
	changeRouteIncrement(msg->data);
}

void AutoWaypointsChange::callbackLoopNext(const std_msgs::Bool::ConstPtr &msg)
{
	/*if(msg->data == true) loopNext();
	else loopBack();
	killWaypointsNode();
	runWaypointsNode(load_list_[loop_count_][lane_count_]);*/
	changeRouteLoopNext(msg->data);
}

void AutoWaypointsChange::callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
{
	if(run_flag_ == false || load_list_.size() == 0) return;

	bool change_flag = false;//waypoint探索で切り替えフラグがtrueの場合に経路切り替えを行う
	double dt = 0;//現在のwaypoint探索距離

	//waypoint経路探索
	unsigned int search_id = 0;
	geometry_msgs::Point change_pose;
	for(int c = 1; c < msg->waypoints.size(); c++)
	{
		//経路launch情報が存在した場合、launch_nameに代入
		if(msg->waypoints[c].waypoint_param.auto_waypoint_change != 0)
		{
			change_flag = true;
			change_pose = msg->waypoints[c+0].pose.pose.position;
			search_id = c;
			break;
		}
		const geometry_msgs::Point &p1 = msg->waypoints[c-1].pose.pose.position;
		const geometry_msgs::Point &p2 = msg->waypoints[c+0].pose.pose.position;
		dt += waypointDT(p1, p2);
		if(dt > CHANGE_PERMISSION_DISTANCE_ + front_length_) break;//waypoint探索距離が車両前方からDT_MAXまで離れた場合、探索を打ち切り
	}
	std::cout << "DT:" << dt << std::endl;
	std::cout << (int)change_flag << std::endl;
	if(change_flag == true && waypointDT(change_pose, current_pose_.pose.position) < CHANGE_PERMISSION_DISTANCE_ + front_length_)
	{
		//wrote by minamidani
		//stroke clutch?
		if((can502_.clutch == false && can503_.clutch == false) ||
			((can502_.clutch == true || can503_.clutch == true) && current_velocity_.twist.linear.x < VELOCITY_LIMIT_))// && can503_.pedal_displacement < (-170)))// && current_velocity_.twist.linear.x < VELOCITY_LIMIT_)
		{
			if(msg->waypoints[search_id].waypoint_param.id != waypoint_id_)
			{
				/*loopIncrement();
				killWaypointsNode();
				runWaypointsNode(load_list_[loop_count_][lane_count_]);*/
				changeRouteIncrement(true);
				waypoint_id_ = msg->waypoints[search_id].waypoint_param.id;
				/*std_msgs::Bool clutch;
				clutch.data = false;
				pub_502_clutch_.publish(clutch);*/
			}
		}
	}
	else waypoint_id_ = 0;
}

AutoWaypointsChange::AutoWaypointsChange(ros::NodeHandle nh, ros::NodeHandle pnh)
	: nh_(nh)
	, pnh_(pnh)
	, waypoint_id_(0)
	, run_flag_(true)
	, loop_count_(0)
	, lane_count_(0)
	, route_change_time_(ros::Time(0))
{
	pnh_.param<double>("/vehicle_info/front_bumper_to_baselink", front_length_, 4.55);

	pub_auto_route_loop_count_ = nh_.advertise<autoware_msgs::AutoRouteLoopCount>("/auto_route_loop_count", 1, true);
	pub_502_clutch_ = nh_.advertise<std_msgs::Bool>("/microbus/steer_clutch", 1, true);
	pub_astar_avoid_end_ = nh_.advertise<std_msgs::Empty>("/astar_avoid_end", 1, false);

	sub_waypoints_ = nh_.subscribe("/final_waypoints", 10, &AutoWaypointsChange::callbackWaypoints, this);
	sub_can502_ = nh_.subscribe("/microbus/can_receive502", 10, &AutoWaypointsChange::callbackCan502, this);
	sub_can503_ = nh_.subscribe("/microbus/can_receive503", 10, &AutoWaypointsChange::callbackCan503, this);
	sub_current_pose_ = nh_.subscribe("/current_pose", 10, &AutoWaypointsChange::callbackCurrentPose, this);
	sub_current_velocity_ = nh_.subscribe("/current_velocity", 10, &AutoWaypointsChange::callbackCurrentVelocity, this);
	sub_run_flag_ = nh_.subscribe("/auto_waypoint_load", 10, &AutoWaypointsChange::callbackRunFlag, this);
	sub_route_list_ = nh_.subscribe("/auto_route_list", 10, &AutoWaypointsChange::callbackRouteList, this);
	sub_loop_increment_ = nh_.subscribe("/auto_route_increment", 10, &AutoWaypointsChange::callbackIncrement, this);
	sub_loop_next_ = nh_.subscribe("/auto_route_loop_next", 10, &AutoWaypointsChange::callbackLoopNext, this);

	loopCountPublish();
}
}