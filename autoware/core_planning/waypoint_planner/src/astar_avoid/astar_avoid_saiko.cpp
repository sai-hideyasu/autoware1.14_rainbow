/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "waypoint_planner/astar_avoid/astar_avoid_saiko.h"

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

autoware_msgs::AvoidanceLaneInfo way_pose_convert(const geometry_msgs::Pose way_pose, double width)
{
	Eigen::Vector3d po(way_pose.position.x, way_pose.position.y, way_pose.position.z);
	double roll, pitch, yaw;
	geometry_quat_to_rpy(roll, pitch, yaw, way_pose.orientation);
	Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_rot = qua * po;
	po_rot += Eigen::Vector3d(0, width, 0);
	Eigen::Quaterniond qua_rev = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_move = qua_rev * po_rot;

	autoware_msgs::AvoidanceLaneInfo ret;
	ret.x = po_move.x();  ret.y = po_move.y();
	return ret;
}

AstarAvoid::AstarAvoid()
  : nh_()
  , private_nh_("~")
  , closest_waypoint_index_(-1)
  , obstacle_waypoint_index_(-1)
  , closest_local_index_(-1)
  , costmap_initialized_(false)
  , current_pose_initialized_(false)
  , current_velocity_initialized_(false)
  , base_waypoints_initialized_(false)
  , closest_waypoint_initialized_(false)
  , terminate_thread_(false)
{
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 200); std::cout << "size:" << safety_waypoints_size_ << std::endl;
  private_nh_.param<double>("update_rate", update_rate_, 10.0);

  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, false);
  private_nh_.param<double>("avoid_waypoints_velocity", avoid_waypoints_velocity_, 10.0);
  private_nh_.param<double>("avoid_start_velocity", avoid_start_velocity_, 5.0);
  private_nh_.param<double>("replan_interval", replan_interval_, 2.0);
  private_nh_.param<int>("search_waypoints_size", search_waypoints_size_, 50);
  private_nh_.param<int>("search_waypoints_delta", search_waypoints_delta_, 2);
  private_nh_.param<int>("closest_search_size", closest_search_size_, 30);

  //tmp_pub_ = nh_.advertise<std_msgs::String>("astar_tmp", 1, true);
  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  no_curve_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("no_curve_waypoints", 1, true);
  back_waypoint_pub_ = nh_.advertise<autoware_msgs::Lane>("back_waypoint", 1, true);

  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarAvoid::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarAvoid::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &AstarAvoid::currentVelocityCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &AstarAvoid::baseWaypointsCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &AstarAvoid::closestWaypointCallback, this);
  obstacle_waypoint_sub_ = nh_.subscribe("obstacle_waypoint", 1, &AstarAvoid::obstacleWaypointCallback, this);
  avoidance_lane_info_list_sub_ = nh_.subscribe("avoidance_lane_info_list", 1, &AstarAvoid::avoidanceLaneInfoCallback, this);

  rate_ = new ros::Rate(update_rate_);
}

AstarAvoid::~AstarAvoid()
{
  publish_thread_.join();
}

void AstarAvoid::avoidanceLaneInfoCallback(const autoware_msgs::AvoidanceLaneInfoList& msg)
{
  avoidance_lane_info_list_ = msg;
}

void AstarAvoid::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);
  costmap_initialized_ = true;
}

void AstarAvoid::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_pose_global_ = msg;

  if (!enable_avoidance_)
  {
    current_pose_initialized_ = true;
  }
  else
  {
    current_pose_local_.pose = transformPose(
        current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
    current_pose_local_.header.frame_id = costmap_.header.frame_id;
    current_pose_local_.header.stamp = current_pose_global_.header.stamp;
    current_pose_initialized_ = true;
  }
}

void AstarAvoid::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  current_velocity_ = msg;
  current_velocity_initialized_ = true;
}

void AstarAvoid::baseWaypointsCallback(const autoware_msgs::Lane& msg)
{
  static autoware_msgs::Lane prev_base_waypoints;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    base_waypoints_ = msg;
  }
  if (base_waypoints_initialized_)
  {
    // detect waypoint change by timestamp update
    ros::Time t1 = prev_base_waypoints.header.stamp;
    ros::Time t2 = base_waypoints_.header.stamp;
    if ((t2 - t1).toSec() > 1e-3)
    {
      ROS_INFO("Receive new /base_waypoints, reset waypoint index.");
      closest_local_index_ = -1; // reset local closest waypoint
      prev_base_waypoints = base_waypoints_;
    }
  }
  else
  {
    prev_base_waypoints = base_waypoints_;
  }

  base_waypoints_initialized_ = true;
}

void AstarAvoid::closestWaypointCallback(const std_msgs::Int32& msg)
{
  closest_waypoint_index_ = msg.data;

  if (closest_waypoint_index_ == -1)
  {
    closest_local_index_ = -1; // reset local closest waypoint
  }

  closest_waypoint_initialized_ = true;
}

void AstarAvoid::obstacleWaypointCallback(const std_msgs::Int32& msg)
{
  obstacle_waypoint_index_ = msg.data;
}

void AstarAvoid::run()
{
  // check topics
  state_ = AstarAvoid::STATE::INITIALIZING;

  while (ros::ok())
  {
    ros::spinOnce();
    if (checkInitialized())
    {
      break;
    }
    ROS_WARN("Waiting for subscribing topics...");
    ros::Duration(1.0).sleep();
  }

  // main loop
  int end_of_avoid_index = -1;
  ros::WallTime start_plan_time = ros::WallTime::now();
  ros::WallTime start_avoid_time = ros::WallTime::now();

  // reset obstacle index
  obstacle_waypoint_index_ = -1;

  // relaying mode at startup
  state_ = AstarAvoid::STATE::RELAYING;

  // start publish thread
  publish_thread_ = std::thread(&AstarAvoid::publishWaypoints, this);

  while (ros::ok())
  {
    ros::spinOnce();

    // relay mode
    if (!enable_avoidance_)
    {
      rate_->sleep();
      continue;
    }

    // avoidance mode
    bool found_obstacle = (obstacle_waypoint_index_ >= 0);
    bool avoid_velocity = (current_velocity_.twist.linear.x < avoid_start_velocity_ / 3.6);

    // update state
    if (state_ == AstarAvoid::STATE::RELAYING)
    {
      avoid_waypoints_ = base_waypoints_;
      if (found_obstacle)
      {
        ROS_INFO("RELAYING -> STOPPING, Decelerate for stopping");
        state_ = AstarAvoid::STATE::STOPPING;
      }
    }
    else if (state_ == AstarAvoid::STATE::STOPPING)
    {
      bool replan = ((ros::WallTime::now() - start_plan_time).toSec() > replan_interval_);

      if (!found_obstacle)
      {
        ROS_INFO("STOPPING -> RELAYING, Obstacle disappers");
        state_ = AstarAvoid::STATE::RELAYING;
      }
      else if (replan && avoid_velocity)
      {
        ROS_INFO("STOPPING -> PLANNING, Start A* planning");
        state_ = AstarAvoid::STATE::PLANNING;
      }
    }
    else if (state_ == AstarAvoid::STATE::PLANNING)
    {
      start_plan_time = ros::WallTime::now();

      if (planAvoidWaypoints(end_of_avoid_index))
      {
        ROS_INFO("PLANNING -> AVOIDING, Found path");
        {
          std::lock_guard<std::mutex> lock(mutex_);
          avoid_waypoints_publish_ = avoid_waypoints_;
        }
        state_ = AstarAvoid::STATE::AVOIDING;
        start_avoid_time = ros::WallTime::now();
      }
      else
      {
        ROS_INFO("PLANNING -> STOPPING, Cannot find path");
        state_ = AstarAvoid::STATE::STOPPING;
      }
    }
    else if (state_ == AstarAvoid::STATE::AVOIDING)
    {
      bool reached = (getLocalClosestWaypoint(avoid_waypoints_, current_pose_global_.pose, closest_search_size_) > end_of_avoid_index);
      if (reached)
      {
        ROS_INFO("AVOIDING -> RELAYING, Reached goal");
        state_ = AstarAvoid::STATE::RELAYING;
      }
      else if (found_obstacle && avoid_velocity)
      {
        bool replan = ((ros::WallTime::now() - start_avoid_time).toSec() > replan_interval_);
        if (replan)
        {
          ROS_INFO("AVOIDING -> STOPPING, Abort avoiding");
          state_ = AstarAvoid::STATE::STOPPING;
        }
      }
    }

    rate_->sleep();
  }

  terminate_thread_ = true;
}

bool AstarAvoid::checkInitialized()
{
  bool initialized = false;

  // check for relay mode
  initialized = (current_pose_initialized_ && closest_waypoint_initialized_ && base_waypoints_initialized_ &&
                 (closest_waypoint_index_ >= 0));

  // check for avoidance mode, additionally
  if (enable_avoidance_)
  {
    initialized = (initialized && (current_velocity_initialized_ && costmap_initialized_));
  }

  return initialized;
}

bool AstarAvoid::planAvoidWaypoints(int& end_of_avoid_index)
{
  bool found_path = false;
  int closest_waypoint_index = getLocalClosestWaypoint(avoid_waypoints_, current_pose_global_.pose, closest_search_size_);

  // update goal pose incrementally and execute A* search
  for (int i = search_waypoints_delta_; i < static_cast<int>(search_waypoints_size_); i += search_waypoints_delta_)
  {
    // update goal index
    int goal_waypoint_index = closest_waypoint_index + obstacle_waypoint_index_ + i;
    if (goal_waypoint_index >= static_cast<int>(avoid_waypoints_.waypoints.size()))
    {
      break;
    }

    // update goal pose
    goal_pose_global_ = avoid_waypoints_.waypoints[goal_waypoint_index].pose;
    goal_pose_local_.header = costmap_.header;
    goal_pose_local_.pose = transformPose(goal_pose_global_.pose,
                                          getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));

    // initialize costmap for A* search
    astar_.initialize(costmap_);

    // execute astar search
    // ros::WallTime start = ros::WallTime::now();
    found_path = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
    // ros::WallTime end = ros::WallTime::now();

    static ros::Publisher pub = nh_.advertise<nav_msgs::Path>("debug", 1, true);

    // ROS_INFO("Astar planning: %f [s], at index = %d", (end - start).toSec(), goal_waypoint_index);

    if (found_path)
    {
      pub.publish(astar_.getPath());
      end_of_avoid_index = goal_waypoint_index;
      mergeAvoidWaypoints(astar_.getPath(), end_of_avoid_index);
      if (avoid_waypoints_.waypoints.size() > 0)
      {
        ROS_INFO("Found GOAL at index = %d", goal_waypoint_index);
        astar_.reset();
        return true;
      }
      else
      {
        found_path = false;
      }
    }
    astar_.reset();
  }

  ROS_ERROR("Can't find goal...");
  return false;
}

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, int& end_of_avoid_index)
{
  autoware_msgs::Lane current_waypoints = avoid_waypoints_;

  // reset
  avoid_waypoints_.waypoints.clear();

  // add waypoints before start index
  int closest_waypoint_index = getLocalClosestWaypoint(current_waypoints, current_pose_global_.pose, closest_search_size_);
  for (int i = 0; i < closest_waypoint_index; ++i)
  {
    avoid_waypoints_.waypoints.push_back(current_waypoints.waypoints.at(i));
  }

  // set waypoints for avoiding
  for (const auto& pose : path.poses)
  {
    autoware_msgs::Waypoint wp;
    wp.pose.header = avoid_waypoints_.header;
    wp.pose.pose = transformPose(pose.pose, getTransform(avoid_waypoints_.header.frame_id, pose.header.frame_id));
    wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
    wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;       // velocity = const
    avoid_waypoints_.waypoints.push_back(wp);
  }

  // add waypoints after goal index
  for (int i = end_of_avoid_index; i < static_cast<int>(current_waypoints.waypoints.size()); ++i)
  {
    avoid_waypoints_.waypoints.push_back(current_waypoints.waypoints.at(i));
  }

  // update index for merged waypoints
  end_of_avoid_index = closest_waypoint_index + path.poses.size();
}

void AstarAvoid::publishWaypoints()
{
  autoware_msgs::Lane current_waypoints;

  while (!terminate_thread_)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      // select waypoints
      switch (state_)
      {
        case AstarAvoid::STATE::RELAYING:
          current_waypoints = base_waypoints_;
          break;
        case AstarAvoid::STATE::STOPPING:
          // do nothing, keep current waypoints
          break;
        case AstarAvoid::STATE::PLANNING:
          // do nothing, keep current waypoints
          break;
        case AstarAvoid::STATE::AVOIDING:
          current_waypoints = avoid_waypoints_publish_;
          break;
        default:
          current_waypoints = base_waypoints_;
          break;
      }
    }

    autoware_msgs::Lane safety_waypoints, no_curve_waypoints;
    autoware_msgs::Lane back_waypoint;
    safety_waypoints.header = current_waypoints.header;
    safety_waypoints.increment = current_waypoints.increment;
    no_curve_waypoints.header = current_waypoints.header;
    no_curve_waypoints.increment = current_waypoints.increment;

    // push waypoints from closest index
    std::cout << "safety : " << safety_waypoints_size_ << " clo : " << closest_search_size_ << std::endl;
    for (int i = 0; i < safety_waypoints_size_; ++i)
    {
      int index = getLocalClosestWaypoint(current_waypoints, current_pose_global_.pose, closest_search_size_) + i;
      if (index < 0 || static_cast<int>(current_waypoints.waypoints.size()) <= index)
      {
        break;
      }
      //const autoware_msgs::Waypoint& wp = current_waypoints.waypoints[index];
      autoware_msgs::Waypoint& wp = current_waypoints.waypoints[index];
      if(safety_waypoints.waypoints.size() == 0 && index > 0)
      {
        back_waypoint.waypoints.push_back(current_waypoints.waypoints[index-1]);
        back_waypoint.header = current_waypoints.header;
      }
      no_curve_waypoints.waypoints.push_back(wp);

      //?????????????????????????????????????????????
      for(int avoidance_count=0; avoidance_count < avoidance_lane_info_list_.list.size(); avoidance_count++)
      {
        autoware_msgs::AvoidanceLaneInfo info = avoidance_lane_info_list_.list[avoidance_count];
        if(wp.waypoint_param.id == info.waypoint_id)
        {
          //std::cout << "x : " << wp.pose.pose.position.x << "," << info.x << std::endl;
          //std::cout << "y : " << wp.pose.pose.position.y << "," << info.y << std::endl;
          wp.pose.pose.position.x = info.x;
          wp.pose.pose.position.y = info.y;
          break;
        }
      }

      //????????????????????????????????????????????????????????????????????????
      if(avoidance_lane_info_list_.status == autoware_msgs::AvoidanceLaneInfoList::STATUS_AVOIDANCE_GO)//(avoidance_lane_info_list_.move_width != 0)
      {
        const autoware_msgs::AvoidanceLaneInfo &last_info = avoidance_lane_info_list_.list[avoidance_lane_info_list_.list.size()-1];
        if(last_info.waypoint_id < wp.waypoint_param.id)
        {
          autoware_msgs::AvoidanceLaneInfo conv = way_pose_convert(wp.pose.pose, avoidance_lane_info_list_.move_width);
          wp.pose.pose.position.x = conv.x;
          wp.pose.pose.position.y = conv.y;
        }
      }
      safety_waypoints.waypoints.push_back(wp);
    }

    if (safety_waypoints.waypoints.size() > 0)
    {
      if(back_waypoint.waypoints.size() == 0)
      {
        autoware_msgs::Waypoint way;
        way.pose.pose.position.x = way.pose.pose.position.y = way.pose.pose.position.z = 0;
        back_waypoint.waypoints.push_back(way);
        back_waypoint.header = current_waypoints.header;
      }
      safety_waypoints_pub_.publish(safety_waypoints);
      no_curve_waypoints_pub_.publish(no_curve_waypoints);
      back_waypoint_pub_.publish(back_waypoint);
      /*std_msgs::String str_ros;
      std::stringstream str;
      str << safety_waypoints.waypoints[0].waypoint_param.id << "," << back_waypoint.waypoints[0].waypoint_param.id;
      str_ros.data = str.str();
      tmp_pub_.publish(str_ros);*/
    }

    rate_->sleep();
  }
}

tf::Transform AstarAvoid::getTransform(const std::string& from, const std::string& to)
{
  tf::StampedTransform stf;
  try
  {
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return stf;
}

int AstarAvoid::getLocalClosestWaypoint(const autoware_msgs::Lane& waypoints, const geometry_msgs::Pose& pose, const int& search_size)
{
  static autoware_msgs::Lane local_waypoints;  // around self-vehicle
  const int prev_index = closest_local_index_;

  // search in all waypoints if lane_select judges you're not on waypoints
  if (closest_local_index_ == -1)
  {
    closest_local_index_ = getClosestWaypoint(waypoints, pose);
  }
  // search in limited area based on prev_index
  else
  {
    // get neighborhood waypoints around prev_index
    int start_index = std::max(0, prev_index - search_size / 2);
    int end_index = std::min(prev_index + search_size / 2, (int)waypoints.waypoints.size());
    auto start_itr = waypoints.waypoints.begin() + start_index;
    auto end_itr = waypoints.waypoints.begin() + end_index;
    local_waypoints.waypoints = std::vector<autoware_msgs::Waypoint>(start_itr, end_itr);

    // get closest waypoint in neighborhood waypoints
    closest_local_index_ = start_index + getClosestWaypoint(local_waypoints, pose);
  }

  return closest_local_index_;
}
