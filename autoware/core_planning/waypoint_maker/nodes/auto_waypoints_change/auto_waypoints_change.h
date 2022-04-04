#ifndef AUTO_WAYPOINTS_CHANGE
#define AUTO_WAYPOINTS_CHANGE

#include <ros/ros.h>
#include <ros/package.h>
#include <sys/stat.h>
#include <std_msgs/Empty.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/AutoRouteList.h>
#include <autoware_msgs/AutoRouteLoopCount.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <std_msgs/Bool.h>

namespace auto_waypoints_change
{
//経路自動生成クラス
class AutoWaypointsChange
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Publisher pub_auto_route_loop_count_, pub_502_clutch_, pub_astar_avoid_end_;
	ros::Subscriber sub_waypoints_, sub_global_waypoints_;
	ros::Subscriber sub_can502_, sub_can503_, sub_current_pose_, sub_current_velocity_, sub_run_flag_;
	ros::Subscriber sub_route_list_, sub_loop_increment_, sub_loop_decrement_, sub_loop_next_;

	const double CHANGE_PERMISSION_DISTANCE_ = 10;//!<経路更新情報があるwaypointまでの判定距離(m) 車両前方基準
	const double VELOCITY_LIMIT_ = 2;//!<車両速度がこれ以下の場合に経路切り替え(km/h);
	const double ROUTE_LOAD_WAIT_TIME_ = 3;//!<<新しい経路を読み込んでから、この秒数は経路変更を禁止する

	double front_length_;//!<base_linkからの車両前方距離 rosのstatic parameterから取得
	autoware_can_msgs::MicroBusCan502 can502_;//!<車両CANのID502情報 steerのクラッチを確認するため
	autoware_can_msgs::MicroBusCan503 can503_;//!<車両CANのID503情報 steerのクラッチを確認するため
	geometry_msgs::PoseStamped current_pose_;//!<現在の車両位置
	geometry_msgs::TwistStamped current_velocity_;//!<現在の車両速度
	unsigned int waypoint_id_;//!<launchファイル名が見つかったwaypoint_id。見つからない場合は0。launchファイルの複数回実行を防止するための安全弁として使用
	bool run_flag_;//!<このフラグがtrueの場合に自動経路切り替えが行われる
	int loop_count_;//!<現在の周回数
	int lane_count_;//!<現在の周回の選択経路インデックス
	std::vector<std::vector<std::string>> load_list_;//!<周回毎の経路切り替えリスト
	ros::Time route_change_time_;//経路変更を行った時間

	void killWaypointsNode();
	void runWaypointsNode(std::string launch);

	void changeRoute();

	void changeRouteIncrement(const bool increment);

	void changeRouteLoopNext(const bool loop_move);

	void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr &msg);

	void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg);

	void callbackCan502(const autoware_can_msgs::MicroBusCan502::ConstPtr &msg);

	void callbackCan503(const autoware_can_msgs::MicroBusCan503::ConstPtr &msg);

	void callbackRunFlag(const std_msgs::Bool::ConstPtr &msg);

	void callbackRouteList(const autoware_msgs::AutoRouteList::ConstPtr &msg);

	void loopCountPublish();

	void loopIncrement();

	void loopDecrement();

	void loopNext();

	void loopBack();

	void callbackIncrement(const std_msgs::Bool::ConstPtr &msg);

	void callbackLoopNext(const std_msgs::Bool::ConstPtr &msg);

	void callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg);
public:
	AutoWaypointsChange(ros::NodeHandle nh, ros::NodeHandle pnh);
};
}
#endif