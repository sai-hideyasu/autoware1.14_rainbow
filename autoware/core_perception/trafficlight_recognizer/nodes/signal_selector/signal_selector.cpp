#include <ros/ros.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/WaypointParam.h>

class SignalSelector
{
private:
	const int TRAFFIC_LIGHT_RED = 0;
	const int TRAFFIC_LIGHT_GREEN = 1;
	const int TRAFFIC_LIGHT_UNKNOWN = 2;

	const std::string DEFAULT_SIGNAL_TOPIC = "/light_color_yolo";// スラを忘れずに

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber sub_waypoint_param_;
	std::vector<ros::Subscriber> sub_light_color_list_;
	ros::Publisher pub_light_color_;
	std::string signal_select_topic_;
	//std::vector<LightColor> light_color_list_;

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		std::string topic_name = msg->signal_select_topic;
		if(topic_name[0] != '/')
		{
			std::stringstream ss;
			ss << "/" << topic_name;
			signal_select_topic_ = ss.str();
		}
		else signal_select_topic_ = msg->signal_select_topic;
	}

	/*void light_publish(const int sub_number, const autoware_msgs::TrafficLight::ConstPtr &light_msg)
	{
		for(int cou=0; cou<sub_light_color_list_.size(); cou++)
		{
			ros::Subscriber sub = sub_light_color_list_[cou];
			std::cout << signal_select_topic_ << "," << sub.getTopic() << std::endl;
			if(signal_select_topic_ == sub.getTopic())
			{
				pub_light_color_.publish(*light_msg);
				return;
			}
		}
	}*/

	void callbackLightColor1(const autoware_msgs::TrafficLight::ConstPtr &msg)
	{
		if(signal_select_topic_ == sub_light_color_list_[0].getTopic())
			pub_light_color_.publish(*msg);
	}

	void callbackLightColor2(const autoware_msgs::TrafficLight::ConstPtr &msg)
	{
		if(signal_select_topic_ == sub_light_color_list_[1].getTopic())
			pub_light_color_.publish(*msg);
	}

	void callbackLightColor3(const autoware_msgs::TrafficLight::ConstPtr &msg)
	{
		if(signal_select_topic_ == sub_light_color_list_[2].getTopic())
			pub_light_color_.publish(*msg);
	}
public:
	SignalSelector(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, signal_select_topic_(DEFAULT_SIGNAL_TOPIC)
	{
		ros::Subscriber sub_light_color_default = nh_.subscribe(DEFAULT_SIGNAL_TOPIC, 1, &SignalSelector::callbackLightColor1, this);
		ros::Subscriber sub_light_color_id_jousi = nh_.subscribe("light_color_jousi", 1, &SignalSelector::callbackLightColor2, this);
		ros::Subscriber sub_light_color_id_taya = nh_.subscribe("light_color_taya", 1, &SignalSelector::callbackLightColor3, this);
		sub_light_color_list_.push_back(sub_light_color_default);
		sub_light_color_list_.push_back(sub_light_color_id_jousi);
		sub_light_color_list_.push_back(sub_light_color_id_taya);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 1, &SignalSelector::callbackWaypointParam, this);
		pub_light_color_ = nh_.advertise<autoware_msgs::TrafficLight>("/light_color", 1, true);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "signal_selector");
	ros::NodeHandle nh, pnh("~");

	SignalSelector selector(nh, pnh);
	ros::spin();
	return 0;
}