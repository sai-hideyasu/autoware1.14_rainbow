#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/StopperDistance.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/UltrasoundDataList.h>
#include <autoware_config_msgs/ConfigTemporaryStopper.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <visualization_msgs/MarkerArray.h>

struct SelectedLine
{
	bool on_temporary_;
	bool on_signal_;
	bool on_object_;
	bool on_oncoming_;
	int waypoint_index_;
	double deceleration_;
	double fixed_velocity_;
};

double euclideanDistanceXY(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
	double x1 = p1.x,  x2 = p2.x;
	double y1 = p1.y,  y2 = p2.y;
	double xd = x1 - x2,  yd = y1 - y2;
	return std::sqrt(xd*xd + yd*yd);
}

class TemporaryStopper
{
private:
	const int TRAFFIC_LIGHT_RED = 0;
	const int TRAFFIC_LIGHT_GREEN = 1;
	const int TRAFFIC_LIGHT_YELLOW = 10;
	const int TRAFFIC_LIGHT_YELLOW_RED = 11;
	const int TRAFFIC_LIGHT_UNKNOWN = 2;

	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_waypoint_, sub_waypoint_param_, sub_can503_, sub_current_velocity_, sub_current_pose_;
	ros::Subscriber sub_distance_, sub_config_, sub_light_color_, sub_ultrasound_list_, sub_oncoming_permission_;

	ros::Publisher pub_waypoint_, pub_temporary_line_, pub_temporary_distance_, pub_temporary_flag_, pub_temporary_fixed_velocity_;
	ros::Publisher pub_stop_waypoint_id_, pub_oncoming_stop_, pub_can_stroke_cap_, pub_tmp_;

	autoware_config_msgs::ConfigTemporaryStopper config_;
	autoware_can_msgs::MicroBusCan503 can503_;
	bool can503_read_;
	geometry_msgs::TwistStamped current_velocity_;
	geometry_msgs::PoseStamped current_pose_;
	double front_bumper_to_baselink_;
	uint32_t stop_waypoint_id_;
	ros::Time timer_;
	double stop_time_;
	autoware_msgs::StopperDistance distance_;
	double fixed_velocity_;
	int light_color_;//!<信号情報
	autoware_msgs::UltrasoundDataList ultrasound_list_;//!<超音波情報
	uint32_t use_ultrasound_;//超音波を使用するかのフラグ
	uint32_t oncoming_permission_id_;//対向車待ちの停止線の位置
	SelectedLine prev_selected_line_;//前回の停止線情報
	autoware_msgs::WaypointParam waypoint_param_;

	/*autoware_msgs::Lane apply_deceleration(const autoware_msgs::Lane& lane, double deceleration, int start_index,
										   double fixed_vel, double velocity_limit)
	{
		autoware_msgs::Lane l = lane;

		l.waypoints[start_index].twist.twist.linear.x = 0;

		for(int i=start_index-1; i>=0; i--)
		{
			double prev_v = l.waypoints[i+1].twist.twist.linear.x;
			geometry_msgs::Point a = l.waypoints[i+1].pose.pose.position;
			geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
			//distance += hypot(b.x - a.x, b.y - a.y);
			double distance = hypot(b.x - a.x, b.y - a.y);
			double v;
			if(prev_v < 15 / 3.6)
			{
				v = std::sqrt(prev_v * prev_v + 2 * deceleration * distance);
				std::cout << i << "," << v << "," << "," << v * 3.6 << "," << prev_v << "," << deceleration << "," << distance << std::endl;
			}
			else
			{
				v = prev_v + distance * 0.05;
			}
			
			if (v < l.waypoints[i].twist.twist.linear.x)
			{
				if(v > velocity_limit) l.waypoints[i].twist.twist.linear.x = velocity_limit;//vがvelocity_limitを超えている場合は、速度をvelocity_limitにする
				else l.waypoints[i].twist.twist.linear.x = v;
			}
		}

		return l;
	}*/

	autoware_msgs::Lane apply_deceleration(const autoware_msgs::Lane& lane, double deceleration, int start_index,
										   size_t fixed_cnt, double fixed_vel, double velocity_limit, double &stop_distance)
	{
		autoware_msgs::Lane l = lane;
		//std::cout << "fixed cnt:" << fixed_cnt << "," << start_index << "," << lane.waypoints[start_index].waypoint_param.id << "," << fixed_vel << std::endl;
		if (fixed_cnt == 0)
		return l;

		const double vel_th_kmh = 15;
		double square_vel = fixed_vel * fixed_vel;
		double distance_sum = 0;
		double next_v = 0;
		int i;
		for (i = start_index; i < l.waypoints.size(); ++i)
		{
			if (i - start_index < fixed_cnt)
			{
				l.waypoints[i].twist.twist.linear.x = fixed_vel;
				continue;
			}

			geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
			geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
			distance_sum += hypot(b.x - a.x, b.y - a.y);

			double v = sqrt(square_vel + 2 * deceleration * distance_sum);
			next_v = v;

			if (v < l.waypoints[i].twist.twist.linear.x)
			{
				//if(fixed_vel < v) l.waypoints[i].twist.twist.linear.x = v;
				//else l.waypoints[i].twist.twist.linear.x = fixed_vel;
				if(velocity_limit < 0) l.waypoints[i].twist.twist.linear.x = v;//速度の最大値がない場合(velocity_limitがマイナス)
				else if(v > velocity_limit) l.waypoints[i].twist.twist.linear.x = velocity_limit;//vがvelocity_limitを超えている場合は、速度をvelocity_limitにする
				else l.waypoints[i].twist.twist.linear.x = v;
			}
			//else if(reverse_flag == true)//停止線より下側は中断処理をしない
			//	break;

			if(v > vel_th_kmh / 3.6) break;
		}

		double prev_v = next_v;
		double dec = deceleration;
		for(int j=i+1; j<l.waypoints.size(); ++j)
		{
			geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
			geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
			double distance = hypot(b.x - a.x, b.y - a.y);

			double v = prev_v + distance * dec;
			l.waypoints[j].twist.twist.linear.x = v;

			distance_sum += distance;
			dec += 0.1;
		}

		stop_distance = distance_sum;
		return l;
	}

	/*autoware_msgs::Lane apply_decceleration(const autoware_msgs::Lane& lane, double decceleration, int start_index,
										   size_t fixed_cnt, double fixed_vel)
	{
	  autoware_msgs::Lane l = lane;
	  if(decceleration > 0) 
		return l;
	  if (fixed_cnt == 0)
		return l;

	  double square_vel = fixed_vel * fixed_vel;
	  double distance = 0;
	  for (int i = start_index; i > 0; --i)
	  {
		if (i > start_index -fixed_cnt)
		{
			l.waypoints[i].twist.twist.linear.x = fixed_vel;
			continue;
		}

		geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
		geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
		distance += hypot(b.x - a.x, b.y - a.y);
	
		double v = sqrt(square_vel - 2 * decceleration * distance);
		std::cout << v << std::endl;
		if (v < l.waypoints[i].twist.twist.linear.x)
			l.waypoints[i].twist.twist.linear.x = v;
		else
			break;
	  }

	  return l;
	}*/

	SelectedLine stop_line_search(autoware_msgs::Lane& way, int start_index)
	{
		std_msgs::Int32 dis;
		dis.data = -1;

		SelectedLine selected = {false, false, false, false, -1, -1, 0};
		for(int i=start_index; i<way.waypoints.size() || i<config_.search_distance; i++)
		{
			bool ultrasound_flag = false;
			if(way.waypoints[i].waypoint_param.signal_stop_line > 0)
			{
				if(light_color_ == TRAFFIC_LIGHT_RED || light_color_ == TRAFFIC_LIGHT_YELLOW_RED)
				{
					selected.on_signal_ = true;
					if(way.waypoints[i].waypoint_param.signal_stop_line == 1) //左折
						use_ultrasound_ = autoware_msgs::UltrasoundData::POS_RIGHT_SHALLOWBACK | autoware_msgs::UltrasoundData::POS_RIGHT_BACK;
					else if(way.waypoints[i].waypoint_param.signal_stop_line == 2)//右折
						use_ultrasound_ = autoware_msgs::UltrasoundData::POS_LEFT_SHALLOWBACK | autoware_msgs::UltrasoundData::POS_LEFT_BACK;
				}
				else if(use_ultrasound_ != 0)//超音波処理
				{
					for(autoware_msgs::UltrasoundData ud : ultrasound_list_.list)
					{
						if((ud.position_flag & use_ultrasound_) == 0) continue;
						std_msgs::Int32 pub;
						pub.data = (ud.bit_interval);
						pub_tmp_.publish(pub);
						//if(ud.bit_interval > 0 && ud.bit_interval <= 2) ultrasound_flag = true;//０は超音波ビット履歴で１つだけビットが立っている状態
						if(ud.bit_interval != -1) ultrasound_flag = true;//０は超音波ビット履歴で１つだけビットが立っている状態
						way.waypoints[i].waypoint_param.object_stop_line = 1;
						way.waypoints[i].waypoint_param.temporary_fixed_velocity_kmh = 0;
					}
					if(ultrasound_flag == false) use_ultrasound_ = 0;
				}
				//selected.on_signal_ = true;
			}
			if(way.waypoints[i].waypoint_param.temporary_stop_line > 0)
			{
				selected.on_temporary_ = true;
			}
			if(way.waypoints[i].waypoint_param.object_stop_line > 0)
			{
				selected.on_object_ = true;
			}
			if(way.waypoints[i].waypoint_param.oncoming_stop_line > 0)
			{
				if(way.waypoints[i].waypoint_param.id == oncoming_permission_id_)
					selected.on_oncoming_ = true;
			}
			//std::cout << std::boolalpha << selected.on_temporary_ << "," << std::boolalpha << selected.on_signal_ << std::endl;

			//if(way.waypoints[i].waypoint_param.temporary_stop_line > 0
			//	light_color_ == TRAFFIC_LIGHT_RED && way.waypoints[i].waypoint_param.signal_stop_line > 0)
			if(selected.on_temporary_ || selected.on_signal_ || selected.on_object_ || selected.on_oncoming_)
			{
				stop_time_ = way.waypoints[i].waypoint_param.temporary_stop_line;
				//fixed_velocity_ = (selected.on_signal_ == true) ? 0 : way.waypoints[i].waypoint_param.temporary_fixed_velocity / 3.6;
				if(selected.on_signal_ || selected.on_oncoming_) selected.fixed_velocity_ = 0;
				else selected.fixed_velocity_ = std::max(way.waypoints[i].waypoint_param.temporary_fixed_velocity_kmh / 3.6, 0.0);
				/*if(fixed_velocity_ < 0)
				{
					pub_temporary_distance_.publish(dis);
					SelectedLine error = {false, false, false, -1};
					return error;
				}*/

				visualization_msgs::Marker marker;
				marker.header.frame_id = "/map";
				marker.ns = "Temporary Line";
				marker.id = 0;
				marker.type = 1;
				marker.action = 0;
				marker.pose = way.waypoints[i].pose.pose;
				marker.pose.position.z += 1;
				marker.scale.x = 0.1;
				marker.scale.y = 15;
				marker.scale.z = 2;
				marker.color.r = 0;
				marker.color.g = 0;
				marker.color.b = 1;
				marker.color.a = 0.3;
				marker.frame_locked = true;
				marker.mesh_use_embedded_materials = false;
				marker.lifetime = ros::Duration(1.0);
				visualization_msgs::MarkerArray array;
				array.markers.push_back(marker);
				pub_temporary_line_.publish(array);

				if(selected.fixed_velocity_ <= 0) dis.data = i;
				else dis.data = -1;
				pub_temporary_distance_.publish(dis);
				/*std_msgs::Float64 msg_fixed_velocity;
				msg_fixed_velocity.data = selected.fixed_velocity_; 
				pub_temporary_fixed_velocity_.publish(msg_fixed_velocity);*/

				selected.waypoint_index_ = i;
				selected.deceleration_ = way.waypoints[i].waypoint_param.temporary_deceleration;
				return selected;
			}
		}

		oncoming_permission_id_ = 0;
		pub_temporary_distance_.publish(dis);
		return selected;
	}

	//超音波判定
	/*void ultrasuondCheck(autoware_msgs::Lane &lane)
	{
		/*if(current_velocity_.twist.linear.x > 5 / 3.6) return;
		if(current_pose_.pose.position.x == DBL_MIN) return;

		double stop_distance = -1;//停止線を設定する位置
		for(const autoware_msgs::UltrasoundData &info : ultrasound_list_.list)
		{
			if(info.position_flag == autoware_msgs::UltrasoundData::POS_FRONT_CENTER)
			{
				//超音波周波数毎に停止線距離を設定
				if(info.bit_interval == 1) stop_distance = front_bumper_to_baselink_ + 0.00;
				if(info.bit_interval == 2) stop_distance = front_bumper_to_baselink_ + 0.00;
				if(info.bit_interval == 3) stop_distance = front_bumper_to_baselink_ + 0.05;
				if(info.bit_interval == 4) stop_distance = front_bumper_to_baselink_ + 0.10;
				if(info.bit_interval == 5) stop_distance = front_bumper_to_baselink_ + 0.15;
				if(info.bit_interval == 6) stop_distance = front_bumper_to_baselink_ + 0.25;
				if(info.bit_interval >  6) stop_distance = front_bumper_to_baselink_ + 0.5;
				if(info.bit_interval == 0) stop_distance = front_bumper_to_baselink_ + 0.5;
				break;
			}
		}
		if(stop_distance == -1) return;

		double dis_sum = euclideanDistanceXY(current_pose_.pose.position, lane.waypoints[0].pose.pose.position);
		for(int cou=1; cou<lane.waypoints.size(); cou++)
		{
			autoware_msgs::WaypointParam &param = lane.waypoints[cou].waypoint_param;
			dis_sum += euclideanDistanceXY(lane.waypoints[cou].pose.pose.position, lane.waypoints[cou-1].pose.pose.position);
			if(stop_distance <= dis_sum)
			{
				param.object_stop_line = 1;
				param.temporary_fixed_velocity = 0;
				param.stop_line_adjustment = stop_distance - dis_sum;
				break;
			}
		}

		return;
	}*/

	autoware_msgs::Lane apply_stopline_deceleration(const autoware_msgs::Lane& lane, double decceleration,
													int ahead_cnt, int behind_cnt)
	{
		std_msgs::Bool oncoming_msg;//対向車で停止したかのフラグをpublish
		oncoming_msg.data = false;

		autoware_msgs::Lane new_lane = lane;

		//ultrasuondCheck(new_lane);

		std_msgs::Int32 flag;

		//経路情報の停止フラグを探索して停止線フラグの位置を取得
		SelectedLine stop_index = stop_line_search(new_lane, 0); //std::cout << stop_index.waypoint_index_ << std::endl;
		if(!stop_index.on_signal_ && !stop_index.on_object_ && !stop_index.on_oncoming_ && stop_index.on_temporary_
			&& new_lane.waypoints[stop_index.waypoint_index_].waypoint_param.temporary_fixed_velocity_kmh != 0)
		{
			SelectedLine stop_index2 = stop_line_search(new_lane, stop_index.waypoint_index_+1);
			if(stop_index2.waypoint_index_ != -1)
			{
				if((stop_index2.on_signal_ || stop_index2.on_object_ || stop_index2.on_oncoming_)
					&& (stop_index2.waypoint_index_ - stop_index.waypoint_index_) <= 20)
				{
					std::cout << "stop_ind," << stop_index2.waypoint_index_ << "," << stop_index.waypoint_index_ << std::endl;
					new_lane.waypoints[stop_index.waypoint_index_].waypoint_param.temporary_stop_line = 0;
					stop_index = stop_index2;
				}
			}
		}
		prev_selected_line_ = stop_index;
		fixed_velocity_ = stop_index.fixed_velocity_;
		std_msgs::Float64 msg_fixed_velocity;
		msg_fixed_velocity.data = fixed_velocity_; 
		pub_temporary_fixed_velocity_.publish(msg_fixed_velocity);

		if(!stop_index.on_signal_ && !stop_index.on_temporary_ && !stop_index.on_object_ && !stop_index.on_oncoming_)
		{
			std_msgs::Int16 cap_msg;
			cap_msg.data = 500;
			pub_can_stroke_cap_.publish(cap_msg);

			stop_waypoint_id_ = 0;
			flag.data = 0;//そのまま
			pub_temporary_flag_.publish(flag);
			timer_ = ros::Time::now();
			std_msgs::Int32 stop_id_ret;
			stop_id_ret.data = -1;
			pub_stop_waypoint_id_.publish(stop_id_ret);
			pub_oncoming_stop_.publish(oncoming_msg);
			return new_lane;
		}

		const autoware_msgs::Waypoint &stop_way = new_lane.waypoints[stop_index.waypoint_index_];
		std_msgs::Int32 stop_id_ret;
		stop_id_ret.data = stop_way.waypoint_param.id;
		pub_stop_waypoint_id_.publish(stop_id_ret);
		
		if(can503_.clutch == false) timer_ = ros::Time(0);//手動運転の場合はTimer処理はしない

		//一時停止処理の判定
		if(stop_index.on_temporary_ == true)
		{
			ros::Time now_time = ros::Time::now();
			if(timer_ < now_time)
			{
				//std::cout << "distance_1 : " << distance_ << std::endl;
				if(stop_waypoint_id_ == stop_way.waypoint_param.id)
				{
					flag.data = 2;
					pub_temporary_flag_.publish(flag);
					return new_lane;//停止判定終了
				}

				stop_waypoint_id_ = 0;
				//if(can_.velocity_actual <= config_.stop_speed_threshold)   //幅を持たせなくてよいか？
				if(distance_.send_process == autoware_msgs::StopperDistance::TEMPORARY_STOPPER)//停止線の種類が一時停止線か
				{
					if(distance_.distance <= 3 && distance_.distance >=0)//停止線までの距離判定
					{
						if(current_velocity_.twist.linear.x <= config_.stop_speed_threshold || can503_.clutch == false)//スピードの0判定
						{
							stop_waypoint_id_ = stop_way.waypoint_param.id;
							ros::Duration ros_stop_time(stop_time_);
							timer_ = now_time + ros_stop_time;

							std::cout << "time : " << timer_.sec << "," << now_time.sec << std::endl;
						}
						if(can503_.clutch == false)
						{
							timer_ = ros::Time(0);//手動運転の場合はTimer処理はしない
						}
						if(distance_.fixed_velocity != 0)
						{
							stop_waypoint_id_ = stop_way.waypoint_param.id;
							timer_ = ros::Time(0);
						}
					}
				}
				flag.data = 1;//停止判定あり
				pub_temporary_flag_.publish(flag);
			}
		}
		/*else if(stop_index.select_ == SELECTED_SIGNAL)
		{
			//一時停止の情報をキャンセル
			timer_ = ros::Time(0);
			stop_waypoint_id_ = 0;
		}*/

		//else if(can503_read_ == true && can503_.clutch == false) timer_ = ros::Time(0);//停止状態でクラッチが切れている（手動）場合、停止処理を強制終了

		// front バンパーが一時停止線にあるときに、ベースリンクに一番近い2個のwaypointのうち前のもの。
		// ≒ stop_index - 5
		double stop_line_to_baselink = 0;
		int stop_line_to_baselink_index;
		for(int cou=stop_index.waypoint_index_; cou>=1; cou--)
		{
			stop_line_to_baselink += euclideanDistanceXY(new_lane.waypoints[cou].pose.pose.position, new_lane.waypoints[cou-1].pose.pose.position);
			if(stop_line_to_baselink >= front_bumper_to_baselink_)
			{
				stop_line_to_baselink_index = cou;//+1;
				break;
			}
		}
		if(stop_line_to_baselink < front_bumper_to_baselink_ && fixed_velocity_ <= 0)
		{
			//autoware_msgs::Lane l = lane;
			new_lane.waypoints[0].twist.twist.linear.x = 0;
			pub_oncoming_stop_.publish(oncoming_msg);
			return new_lane;
		}

		
		//double limit_vel = (stop_index.on_object_ == true && stop_index.on_signal_ == false) ? stop_way.waypoint_param.temporary_fixed_velocity / 3.6 : config_.velocity_limit / 3.6;
		double limit_vel;
		if(stop_index.on_signal_ == true) limit_vel = config_.velocity_limit / 3.6;
		else if(stop_index.on_object_ == true) limit_vel = config_.velocity_limit / 3.6;//std::max(10.0 / 3.6, stop_way.waypoint_param.temporary_fixed_velocity / 3.6);
		else if(stop_index.on_temporary_ == true) limit_vel = config_.velocity_limit / 3.6;
		else if(stop_index.on_oncoming_ == true)
		{
			config_.velocity_limit / 3.6;
			oncoming_msg.data = true;
		}
		else limit_vel = config_.velocity_limit / 3.6;

		double dec = (stop_index.deceleration_ < 0) ? decceleration : stop_index.deceleration_;
		double stop_distance;
		//std::cout << "deceleration," << dec << std::endl;
		//new_lane = apply_deceleration(new_lane, dec, stop_line_to_baselink_index, fixed_velocity_, limit_vel);
		new_lane = apply_deceleration(new_lane, dec, stop_line_to_baselink_index, behind_cnt + 1, fixed_velocity_, -1, stop_distance);
		std::reverse(new_lane.waypoints.begin(), new_lane.waypoints.end());
		int reverse_stop_index = new_lane.waypoints.size() - stop_line_to_baselink_index - 1;
		new_lane = apply_deceleration(new_lane, dec, reverse_stop_index, ahead_cnt + 1, fixed_velocity_, limit_vel, stop_distance);
		std::reverse(new_lane.waypoints.begin(), new_lane.waypoints.end());

		pub_oncoming_stop_.publish(oncoming_msg);

		std_msgs::Int16 cap_msg;
		if(stop_distance <= 40 && stop_distance >=20) cap_msg.data = 250;
		else if(stop_distance <= 20) cap_msg.data = 200;
		else cap_msg.data = 500;
		pub_can_stroke_cap_.publish(cap_msg);

		return new_lane;
	}


	void callbackCan503(const autoware_can_msgs::MicroBusCan503& msg)
	{
		can503_ = msg;
		can503_read_ = true;
	}

	void callbackDistance(const autoware_msgs::StopperDistance& msg)
	{
		distance_ = msg;
	}

	void callbackCurrentVelocity(const geometry_msgs::TwistStamped& msg)
	{
		current_velocity_ = msg;
	}

	void callbackCurrentPose(const geometry_msgs::PoseStamped& msg)
	{
		current_pose_ = msg;
	}

	void callbackConfig(const autoware_config_msgs::ConfigTemporaryStopper& msg)
	{
		config_ = msg;
		fixed_velocity_ = config_.fixed_velocity / 3.6;
		//std::cout << config_.fixed_velocity << std::endl;

		std_msgs::Float64 fixed_vel;
		fixed_vel.data = fixed_velocity_;
		pub_temporary_fixed_velocity_.publish(fixed_vel);
	}

	void callbackWaypoint(const autoware_msgs::Lane& msg)
	{
		autoware_msgs::Lane lane = apply_stopline_deceleration(msg, config_.deceleration,
									config_.number_of_zeros_ahead, config_.number_of_zeros_behind);
		pub_waypoint_.publish(lane);
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam& msg)
	{
		waypoint_param_ = msg;
	}

	void callbackLightColor(const autoware_msgs::TrafficLight& msg)
	{
		light_color_ = msg.traffic_light;
	}

	void callbackUltrasoundList(const autoware_msgs::UltrasoundDataList& msg)
	{
		ultrasound_list_ = msg;
	}

	void callbackOncomingPermission(const std_msgs::UInt32 &msg)
	{
		oncoming_permission_id_ = msg.data;
	}
public:
	TemporaryStopper(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: stop_waypoint_id_(100)//0
		, stop_time_(5.0)
		, fixed_velocity_(0)
		, can503_read_(false)
		, light_color_(TRAFFIC_LIGHT_UNKNOWN)
		, use_ultrasound_(0)
		, oncoming_permission_id_(0)
	{
		nh_ = nh;  private_nh_ = p_nh;

		/*config_.search_distance = 7;
		config_.deceleration = 1;
		config_.number_of_zeros_ahead = 0;
		config_.number_of_zeros_behind = 5;
		config_.stop_speed_threshold = 0.018;*/

		timer_ = ros::Time::now();

		nh_.param<double>("/vehicle_info/front_bumper_to_baselink", front_bumper_to_baselink_, 4.55);

		pub_waypoint_ = nh_.advertise<autoware_msgs::Lane>("/temporary_stop_waypoints", 1);
		//pub_waypoint_ = nh_.advertise<autoware_msgs::Lane>("/final_waypoints", 1);
		pub_temporary_line_ = nh_.advertise<visualization_msgs::MarkerArray>("/temporary_line", 1);
		pub_temporary_distance_ = nh_.advertise<std_msgs::Int32>("/temporary_distance", 1);
		pub_temporary_flag_ = nh_.advertise<std_msgs::Int32>("/temporary_flag", 1);
		pub_temporary_fixed_velocity_ = nh_.advertise<std_msgs::Float64>("/temporary_fixed_velocity", 1, true);
		pub_stop_waypoint_id_ = nh_.advertise<std_msgs::Int32>("/stop_waypoint_id", 1);
		pub_oncoming_stop_ = nh_.advertise<std_msgs::Bool>("/oncoming_stop", 1);
		pub_can_stroke_cap_ = nh_.advertise<std_msgs::Int16>("/temporary_stopper/accel_stroke_cap", 1);
		pub_tmp_ = nh_.advertise<std_msgs::Int32>("/temporary_stopper/tmp", 1);

		//sub_waypoint_ = nh_.subscribe("/curve_appropriate_waypoints", 1, &TemporaryStopper::callbackWaypoint, this);
		sub_waypoint_ = nh_.subscribe("/car_tracking_waypoints", 1, &TemporaryStopper::callbackWaypoint, this);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 1, &TemporaryStopper::callbackWaypointParam, this);
		sub_can503_ = nh_.subscribe("/microbus/can_receive503", 1, &TemporaryStopper::callbackCan503, this);
		sub_current_velocity_ = nh_.subscribe("/current_velocity", 1, &TemporaryStopper::callbackCurrentVelocity, this);
		sub_current_pose_ = nh_.subscribe("/current_pose", 1, &TemporaryStopper::callbackCurrentPose, this);
		sub_distance_ = nh_.subscribe("/stopper_distance", 1, &TemporaryStopper::callbackDistance, this);
		sub_config_ = nh_.subscribe("/config/temporary_stopper", 1, &TemporaryStopper::callbackConfig, this);
		sub_light_color_ = nh_.subscribe("/light_color", 1, &TemporaryStopper::callbackLightColor, this);
		sub_ultrasound_list_ = nh_.subscribe("/ultrasound_data_list", 1, &TemporaryStopper::callbackUltrasoundList, this);
		sub_oncoming_permission_ = nh_.subscribe("/oncoming_permission", 1, &TemporaryStopper::callbackOncomingPermission, this);
		distance_.distance = 1000;

		std_msgs::Float64 fixed_vel;
		fixed_vel.data = fixed_velocity_;
		pub_temporary_fixed_velocity_.publish(fixed_vel);

		current_pose_.pose.position.x = current_pose_.pose.position.y = current_pose_.pose.position.z = DBL_MIN;
		current_velocity_.twist.linear.x = 0;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "temporary_stopper");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	TemporaryStopper ts(nh, private_nh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
