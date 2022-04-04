#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <autoware_config_msgs/ConfigLocalizerSmoothTransition.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/DifferenceToWaypointDistance.h>
#include <autoware_msgs/LocalizerCorrect.h>

double sign(double A){
    return (A>0)-(A<0);
}

class LocalizerSmoothTransition
{
private:
	//const double CORRECT_HZ = 10;//!<localizer補正値をpublishするタイマ−の周期
	//const double RETURN_HZ = 10;//!<localizer変更後に微調整したlocalizerを元の状態に戻すタイマーの周期

	const double angle_th_deg_ = 1;//!<比較するlocalizerのangle(deg)がこの差以下になったら切り替え角度判定OK
	const double distance_th_ = 0.02;//!<比較するlocalizerのdistanceがこの距離以下になったら切り替え距離判定OK

	const int LOCALIZER_NONE = -1;
	const int LOCALIZER_NDT = 0;
	const int LOCALIZER_GNSS = 1;

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Publisher pub_smooth_transition_correct_ndt_;//!<gnss localizerノードに送る補正値
	ros::Publisher pub_smooth_transition_correct_gnss_;//!<gnss localizerノードに送る補正値
	ros::Publisher pub_localizer_select_;//!<localizerの変更を通知
	ros::Publisher pub_smooth_transition_correct_diff_;

	ros::Subscriber sub_config_;//!<このノードのconfig
	ros::Subscriber sub_localizer_select_num_;//!<現在のlocalizerの種類
	ros::Subscriber sub_localizer_smooth_chage_;//!<localizer切り替えを行う簡易窓口
	ros::Subscriber sub_waypoint_param_;//!<現在のwaypoint情報
	ros::Subscriber sub_difference_to_waypoint_distance_ndt_;//!<現在のndt localizerと経路の差
	ros::Subscriber sub_localizer_ndt_correct_;//!<現在のndt localizerの補正値
	ros::Subscriber sub_difference_to_waypoint_distance_gnss_;//!<現在のgnss localizerと経路の差
	ros::Subscriber sub_localizer_gnss_correct_;//!<現在のgnss localizerの補正値

	ros::Timer correct_timer_;//!<localizer変更指令が来たら動作する、変更先localizeを微調整するループタイマー
	ros::Timer return_timer_;//!<localizerが変更されたら、微調整したlocalizerを元の状態に戻すループタイマー

	//double angle_correct_add_deg_;// = 2.0 / CORRECT_HZ;//!<１タイマー周期に増減するangle(deg)補正値
	//double distance_correct_add_;// = 0.1 / CORRECT_HZ;//!<１タイマー周期に増減するdistance補正値

	autoware_config_msgs::ConfigLocalizerSmoothTransition config_;//!<このノードのconfig
	int localizer_select_num_;//!<現在のlocalizerの種類
	int localizer_select_change_;//!<現在のlocalizer変更指令

	autoware_msgs::DifferenceToWaypointDistance distance_ndt_;//!<現在のndt localizerと経路の差
	bool read_distance_ndt_;//!<distance_ndt_をsubscribeしたか？
	autoware_msgs::LocalizerCorrect correct_ndt_localizer_;//!<現在のndt localizerの補正値
	autoware_msgs::LocalizerCorrect correct_ndt_localizer_change_before_;//!<localizer切り替え前のcorrect_ndt_localizer_
	autoware_msgs::LocalizerCorrect correct_ndt_localizer_change_correct_send_;//!<localizer微調整時にndtに通知する補正値

	autoware_msgs::DifferenceToWaypointDistance distance_gnss_;//!<現在のgnss localizerと経路の差
	bool read_distance_gnss_;//!<distance_gnss_をsubscribeしたか？
	autoware_msgs::LocalizerCorrect correct_gnss_localizer_;//!<現在のgnss localizerの補正値
	autoware_msgs::LocalizerCorrect correct_gnss_localizer_change_before_;//!<localizer切り替え前のcorrect_gnss_localizer_
	autoware_msgs::LocalizerCorrect correct_gnss_localizer_change_correct_send_;//!<localizer微調整時にgnssに通知する補正値

	autoware_msgs::LocalizerCorrect correct_localizer_change_return_send_;//!<localizer復帰時に通知する補正値

	void callbackConfig(const autoware_config_msgs::ConfigLocalizerSmoothTransition::ConstPtr &msg)
	{
		correct_timer_ = nh_.createTimer(ros::Duration(1.0 / msg->correct_hz), &LocalizerSmoothTransition::callbackCorrectTimer, this);
		correct_timer_.stop();
		return_timer_ = nh_.createTimer(ros::Duration(1.0/ msg->return_hz), &LocalizerSmoothTransition::callbackReturnTimer, this);
		return_timer_.stop();
		config_ = *msg;
	}

	void callbackLocalizerSelectNum(const std_msgs::Int32::ConstPtr &msg)
	{
		localizer_select_num_ = msg->data;
	}

	//切り替えたいlocalizer指令がある場合は、localizer_select_change_に切り替え情報を保持
	//現在のlocalizerと切り替えlocalizerが同じ場合はなにもしない
	void localizerSelectChange(const int8_t trigger)
	{
		if(read_distance_ndt_ == false || read_distance_gnss_ == false) return;

		//切り替えたいlocalizer指令がある場合は、localizer_select_change_に切り替え情報を保持
		//現在のlocalizerと切り替えlocalizerが同じ場合はなにもしない
		if(trigger != LOCALIZER_NONE && localizer_select_num_ !=  trigger)
		{
			if(trigger == LOCALIZER_NDT)
			{
				localizer_select_change_ = LOCALIZER_NDT;
				correct_ndt_localizer_change_before_ = correct_ndt_localizer_;
				correct_ndt_localizer_change_correct_send_ = correct_ndt_localizer_change_before_;
				correct_timer_.start();
			}
			else if(trigger == LOCALIZER_GNSS)
			{
				localizer_select_change_ = LOCALIZER_GNSS;
				correct_gnss_localizer_change_before_ = correct_gnss_localizer_;
				correct_gnss_localizer_change_correct_send_ = correct_gnss_localizer_change_before_;
				correct_timer_.start();
			}
		}
	}

	void callbackLocalizerSmoothChange(const std_msgs::Int8::ConstPtr &msg)
	{
		localizerSelectChange(msg->data);
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		localizerSelectChange(msg->localizer_change_trigger);
	}

	void callbackDifferenceToWaypointDistanceNdt(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		distance_ndt_ = *msg;
		read_distance_ndt_ = true;
	}

	void callbackDifferenceToWaypointDistanceGnss(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		distance_gnss_ = *msg;
		read_distance_gnss_ = true;
	}

	void callbackCorrectNdt(const autoware_msgs::LocalizerCorrect::ConstPtr &msg)
	{
		correct_ndt_localizer_ = *msg;
	}

	void callbackCorrectGnss(const autoware_msgs::LocalizerCorrect::ConstPtr &msg)
	{
		correct_gnss_localizer_ = *msg;
	}

	void callbackCorrectTimer(const ros::TimerEvent& e)
	{
		bool first_check = true;
		if(read_distance_ndt_ == false || read_distance_gnss_ == false) first_check = false;
		if(localizer_select_num_ == LOCALIZER_NONE) first_check = false;
		if(localizer_select_num_ == localizer_select_change_) first_check = false;
		if(first_check == false)
		{
			correct_timer_.stop();
			return_timer_.stop();
			localizer_select_change_ = LOCALIZER_NONE;
		}

		//distanceとangleの差分を取得
		const double distance_diff = distance_ndt_.baselink_distance - distance_gnss_.baselink_distance;
		const double angle_diff = (distance_ndt_.baselink_angular - distance_gnss_.baselink_angular) * 180 / M_PI;
		autoware_msgs::LocalizerCorrect diff_pub;
		diff_pub.distance_correct = distance_diff;
		diff_pub.angle_correct_deg = angle_diff;
		pub_smooth_transition_correct_diff_.publish(diff_pub);

		//distanceとangleの差分がしきい値以下ならlocalizerの切り替えを実行
		if(std::abs(distance_diff) <= distance_th_ && std::abs(angle_diff) <= angle_th_deg_)
		{
			std_msgs::Int32 select;
			select.data = localizer_select_change_;
			pub_localizer_select_.publish(select);
			correct_timer_.stop();
			return_timer_.start();
			if(localizer_select_change_ == LOCALIZER_NDT)
				correct_localizer_change_return_send_ = correct_ndt_localizer_change_correct_send_;
			else if(localizer_select_change_ == LOCALIZER_GNSS)
				correct_localizer_change_return_send_ = correct_gnss_localizer_change_correct_send_;
			return;
		}

		int distance_add_sign = (sign(distance_diff) < 0) ? 1 : -1;
		int angle_add_sign = (sign(angle_diff) < 0) ? 1 : -1;
		double angle_correct_add_deg = config_.angle_return_1second_deg / config_.correct_hz;
		double distance_correct_add = config_.y_distance_return_1second / config_.correct_hz;
		if(localizer_select_change_ == LOCALIZER_NDT)
		{
			correct_ndt_localizer_change_correct_send_.header.stamp = ros::Time::now();
			correct_ndt_localizer_change_correct_send_.distance_correct += distance_add_sign * distance_correct_add;
			correct_ndt_localizer_change_correct_send_.angle_correct_deg += angle_add_sign * angle_correct_add_deg;			
			pub_smooth_transition_correct_ndt_.publish(correct_ndt_localizer_change_correct_send_);
		}
		else if(localizer_select_change_ == LOCALIZER_GNSS)
		{
			correct_gnss_localizer_change_correct_send_.header.stamp = ros::Time::now();
			correct_gnss_localizer_change_correct_send_.distance_correct += distance_add_sign * distance_correct_add;
			correct_gnss_localizer_change_correct_send_.angle_correct_deg += angle_add_sign * angle_correct_add_deg;			
			pub_smooth_transition_correct_gnss_.publish(correct_gnss_localizer_change_correct_send_);
		}
	}

	void callbackReturnTimer(const ros::TimerEvent& e)
	{
		double angle_correct_deg_send;//微調整タイマー終了時に送った調整値(angle)
		double angle_correct_deg_before;//微調整タイマー開始前の調整値(angle)
		double distance_correct_send;//微調整タイマー終了時に送った調整値(distance)
		double distance_correct_before;//微調整タイマー開始前の調整値(distance)

		if(localizer_select_change_ == LOCALIZER_NDT)
		{
			angle_correct_deg_send = correct_ndt_localizer_change_correct_send_.angle_correct_deg;
			angle_correct_deg_before = correct_ndt_localizer_change_before_.angle_correct_deg;
			distance_correct_send = correct_ndt_localizer_change_correct_send_.distance_correct;
			distance_correct_before = correct_ndt_localizer_change_before_.distance_correct;
		}
		else
		{
			angle_correct_deg_send = correct_gnss_localizer_change_correct_send_.angle_correct_deg;
			angle_correct_deg_before = correct_gnss_localizer_change_before_.angle_correct_deg;
			distance_correct_send = correct_gnss_localizer_change_correct_send_.distance_correct;
			distance_correct_before = correct_gnss_localizer_change_before_.distance_correct;
		}

		double angle_diff_deg = angle_correct_deg_before - angle_correct_deg_send;//復帰ループ開始時の戻す角度
		double distance_diff = distance_correct_before - distance_correct_send;//復帰ループ開始時の戻す距離
		int sign_angle_diff_deg = sign(angle_diff_deg);//戻す角度の方向(０ならangle_diff_degが０)
		int sign_distance_diff = sign(distance_diff);//戻す距離の方向(０ならdistance_diffが０)

		double return_angle_deg = sign_angle_diff_deg * config_.angle_return_1second_deg / config_.return_hz;//このループでのangle戻し値
		double next_angle_deg = correct_localizer_change_return_send_.angle_correct_deg + return_angle_deg;//このループでのpublishするangle correct値
		
		//angleの戻す方向を越えた値の場合は、微調整直前の値を入れる
		if(sign_angle_diff_deg == -1)//マイナス
		{
			if(next_angle_deg > angle_correct_deg_before)
				correct_localizer_change_return_send_.angle_correct_deg = next_angle_deg;
			else
				correct_localizer_change_return_send_.angle_correct_deg = angle_correct_deg_before;
		}
		else if(sign_angle_diff_deg == 1)//プラス
		{
			if(next_angle_deg < angle_correct_deg_before)
				correct_localizer_change_return_send_.angle_correct_deg = next_angle_deg;
			else
				correct_localizer_change_return_send_.angle_correct_deg = angle_correct_deg_before;
		}
		else//同値
			correct_localizer_change_return_send_.angle_correct_deg = angle_correct_deg_before;

		double return_distance = sign_distance_diff * config_.y_distance_return_1second / config_.return_hz;//このループでのdistance戻し値
		double next_distance = correct_localizer_change_return_send_.distance_correct + return_distance;//このループでのpublishするdistance correct値

		//distanceの戻す方向を越えた値の場合は、微調整直前の値を入れる
		if(sign_distance_diff == -1)//マイナス(左方向)
		{
			if(next_distance > distance_correct_before)
				correct_localizer_change_return_send_.distance_correct = next_distance;
			else
				correct_localizer_change_return_send_.distance_correct = distance_correct_before;
		}
		else if(sign_distance_diff == 1)//プラス(右方向)
		{
			if(next_distance < distance_correct_before)
				correct_localizer_change_return_send_.distance_correct = next_distance;
			else
				correct_localizer_change_return_send_.distance_correct = distance_correct_before;
		}
		else//同値
			correct_localizer_change_return_send_.distance_correct = distance_correct_before;

		correct_localizer_change_return_send_.header.stamp = e.current_real;
		if(localizer_select_change_ == LOCALIZER_NDT)
			pub_smooth_transition_correct_ndt_.publish(correct_localizer_change_return_send_);
		else if(localizer_select_change_ == LOCALIZER_GNSS)
			pub_smooth_transition_correct_gnss_.publish(correct_localizer_change_return_send_);

		if(correct_localizer_change_return_send_.angle_correct_deg == angle_correct_deg_before &&
			correct_localizer_change_return_send_.distance_correct == distance_correct_before)
		{
			//終了判定
			return_timer_.stop();
			localizer_select_change_ = LOCALIZER_NONE;
		}
	}
public:
	LocalizerSmoothTransition(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, localizer_select_num_(LOCALIZER_NONE)
		, localizer_select_change_(LOCALIZER_NONE)
		//, angle_correct_add_deg_(2.0/10)
		//, distance_correct_add_(0.1/10)
		, read_distance_ndt_(false)
		, read_distance_gnss_(false)
	{
		pub_smooth_transition_correct_ndt_ = nh_.advertise<autoware_msgs::LocalizerCorrect>("/smooth_transition_correct_ndt", 10, false);
		pub_smooth_transition_correct_gnss_ = nh_.advertise<autoware_msgs::LocalizerCorrect>("/smooth_transition_correct_gnss", 10, false);
		pub_localizer_select_ = nh_.advertise<std_msgs::Int32>("/localizer_select", 10, false);
		pub_smooth_transition_correct_diff_ = nh_.advertise<autoware_msgs::LocalizerCorrect>("/smooth_transition_correct_diff", 10, false);

		sub_config_ = nh_.subscribe<autoware_config_msgs::ConfigLocalizerSmoothTransition>(
			"/config/localizer_smooth_transition", 10, &LocalizerSmoothTransition::callbackConfig, this);
		sub_localizer_select_num_ = nh_.subscribe<std_msgs::Int32>(
			"/localizer_select_num", 10, &LocalizerSmoothTransition::callbackLocalizerSelectNum, this);
		sub_localizer_smooth_chage_ = nh_.subscribe<std_msgs::Int8>(
			"/localizer_smooth_change", 10, &LocalizerSmoothTransition::callbackLocalizerSmoothChange, this);
		sub_waypoint_param_ = nh_.subscribe<autoware_msgs::WaypointParam>(
			"/waypoint_param", 10, &LocalizerSmoothTransition::callbackWaypointParam, this);
		sub_difference_to_waypoint_distance_ndt_ = nh_.subscribe<autoware_msgs::DifferenceToWaypointDistance>(
			"/difference_to_waypoint_distance_ndt", 10, &LocalizerSmoothTransition::callbackDifferenceToWaypointDistanceNdt, this);
		sub_localizer_ndt_correct_ = nh_.subscribe<autoware_msgs::LocalizerCorrect>(
			"/ndt_localizer_correct", 10, &LocalizerSmoothTransition::callbackCorrectNdt, this);
		sub_difference_to_waypoint_distance_gnss_ = nh_.subscribe<autoware_msgs::DifferenceToWaypointDistance>(
			"/difference_to_waypoint_distance_gnss", 10, &LocalizerSmoothTransition::callbackDifferenceToWaypointDistanceGnss, this);
		sub_localizer_gnss_correct_ = nh_.subscribe<autoware_msgs::LocalizerCorrect>(
			"/gnss_localizer_correct", 10, &LocalizerSmoothTransition::callbackCorrectGnss, this);

		config_.angle_return_1second_deg = 1;
		config_.y_distance_return_1second = 0.1;
		config_.correct_hz = 10;
		config_.return_hz = 10;

		correct_timer_ = nh_.createTimer(ros::Duration(1.0/10), &LocalizerSmoothTransition::callbackCorrectTimer, this);
		correct_timer_.stop();
		return_timer_ = nh_.createTimer(ros::Duration(1.0/10), &LocalizerSmoothTransition::callbackReturnTimer, this);
		return_timer_.stop();
		//std::cout << std::boolalpha << correct_timer_.hasStarted() << std::endl;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "localizer_smooth_trasition");
	ros::NodeHandle nh, pnh("~");

	LocalizerSmoothTransition transition(nh, pnh);
	ros::spin();
	return 0;
}