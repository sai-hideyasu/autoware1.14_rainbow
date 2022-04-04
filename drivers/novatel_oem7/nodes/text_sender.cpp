#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sentence_text_sender");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

    ros::Publisher pub_sentence_;

    std::string path = "";
    private_nh.getParam("path", path);
    if(path != "")
    {
        pub_sentence_ = nh.advertise<std_msgs::String>("nmea_sentence", 1);

        std::ifstream ifs(path);
        std::string line;
        ros::Rate rate(10);
        while(getline(ifs,line) && ros::ok())
        {
            ros::spinOnce();
            std_msgs::String msg;
            msg.data = line;
            pub_sentence_.publish(msg);
            std::cout << line << std::endl;
            rate.sleep();
        }
    }
    else
    {
        std::cout << "not path" << std::endl;
    }
    return 0;
}