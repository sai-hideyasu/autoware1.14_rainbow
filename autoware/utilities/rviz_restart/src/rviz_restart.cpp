#include <ros/ros.h>
#include <fstream>
#include <sys/stat.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rviz_restart");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	ros::Rate rate(10);
	while(ros::ok())
	{
		system("ps -ax | grep rviz > ./rviz_survival");

		ros::Rate rate(1);
		rate.sleep();

		bool rviz_flag = false;
		std::ifstream ifs("./rviz_survival", std::ios_base::in);
		if(ifs.is_open())
		{
			while(!ifs.eof())
			{
				char buf[301];
				ifs.getline(buf, 300);
				std::string str(buf);
				if(str.find("rviz/rviz") != std::string::basic_string::npos)
				{
					rviz_flag = true;
					//std::cout << buf << std::endl;
				}
			}
		}

		if(rviz_flag == false)
		{
			system("rosrun rviz rviz &");
			ros::Rate r(2);
			r.sleep();
		}
		rate.sleep();
	}

	system("rm rviz_survival");

	return 0;
}