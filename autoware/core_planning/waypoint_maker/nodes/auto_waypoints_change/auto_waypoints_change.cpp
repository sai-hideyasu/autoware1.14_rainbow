/**
 * @file auto_waypoints_change.cpp
 * @brief 経路変更専用トピックやwaypoints上の経路変更フラグから自動経路変更を行うノードのmain
 * @author Hideyasu Sai(Saitama Institute of Technology)
 * @date 2021/10/04
 * @details ・rosの初期化経路自動切り替え用クラスであるauto_waypoints_change::AutoWaypointsChangeの
 */

#include "auto_waypoints_change.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "auto_waypoints_change");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	auto_waypoints_change::AutoWaypointsChange changer(nh, private_nh);
	ros::spin();
	return 0;
}