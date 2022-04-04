#include "i2v_tcp_reader.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "i2v_tcp_reader");
	ros::NodeHandle nh, pnh("~");

	i2v::I2V_Reader i2v(nh, pnh);
	if(!i2v.isSockOpen())
	{
		std::cout << "TCP/IPができません  IP:" << i2v.getIP() << "  port:" << i2v.getPort() << std::endl;
		i2v.release();
		return 0;
	}
	ros::spin();
	i2v.release();
	return 0;
}