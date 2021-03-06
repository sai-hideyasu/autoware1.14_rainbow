#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <nmea_msgs/Sentence.h>
#include <std_msgs/Float64.h>
#include <string>

namespace {
    ros::Publisher nmea_pub;//, time_diff_pub;
}

void publish(char buf[], const int bufSize)
{
    /*std::stringstream stream(buf);
    std::string str;
    while (std::getline(stream, str, '\n'))
    {
        //ROS_INFO("%c\n\n",str.c_str());
        std::stringstream stream2(str);
        std::string str2;
        std::getline(stream2, str2, '\r');

        if(str.compare(0,1,"$")==0)
        {
            if(str.compare(0,6,"$GPVTG")==0 || str.compare(0,6,"$GPGGA")==0 ||
                    str.compare(0,6,"$PASHR")==0 || str.compare(0,6,"$GPHDT")==0 ||
                    str.compare(0,6,"$GPGST") ||str.compare(0,6,"$GPGSA"))
            {
                //ROS_INFO("%s\n\n",str.c_str());
                //std::cout<<str<<std::endl;
                nmea_msgs::Sentence sentence;
                sentence.header.stamp=ros::Time::now();
                sentence.header.frame_id="gps";
                sentence.sentence=str;
                nmea_pub.publish(sentence);
            }
        }
        else if(str.compare(0,1,"<")==0)
        {
            if(str.compare(0,8,"<BESTPOS")==0 || str.compare(0,12,"<BESTGNSSPOS")==0)
            {
                nmea_msgs::Sentence sentence;
                sentence.header.stamp=ros::Time::now();
                sentence.header.frame_id="gps";
                sentence.sentence=str;
                nmea_pub.publish(sentence);
            }
        }
    }*/

    /*std::string str(buf);
    if(str.compare(0,1,"$")==0)
    {
        if(str.compare(0,6,"$GPVTG")==0 || str.compare(0,6,"$GPGGA")==0 ||
                str.compare(0,6,"$PASHR")==0 || str.compare(0,6,"$GPHDT")==0 ||
                str.compare(0,6,"$GPGST") ||str.compare(0,6,"$GPGSA"))
        {
            //ROS_INFO("%s\n\n",str.c_str());
            //std::cout<<str<<std::endl;
            nmea_msgs::Sentence sentence;
            sentence.header.stamp=ros::Time::now();
            sentence.header.frame_id="gps";
            sentence.sentence=str;
            nmea_pub.publish(sentence);
        }
    }
    else if(str.compare(0,1,"#")==0)
    {
        if(str.compare(0,9,"#BESTPOSA")==0 || str.compare(0,13,"#BESTGNSSPOSA")==0 ||
                str.compare(0,9,"#INSATTXA")==0)
        {
            nmea_msgs::Sentence sentence;
            sentence.header.stamp=ros::Time::now();
            sentence.header.frame_id="gps";
            sentence.sentence=str;
            nmea_pub.publish(sentence);
        }
    }*/

	std::string str(buf);
    nmea_msgs::Sentence sentence;
    sentence.header.stamp=ros::Time::now();
    sentence.header.frame_id="gps";
    sentence.sentence=str;
    nmea_pub.publish(sentence);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"nmea_serial");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string port;
    int baud=0;
    private_nh.getParam("port", port);
    private_nh.getParam("baud", baud);
    if(baud == 110) baud = B110;
    else if(baud == 300) baud = B300;
    else if(baud == 1200) baud = B1200;
    else if(baud == 2400) baud = B2400;
    else if(baud == 4800) baud = B4800;
    else if(baud == 9600) baud = B9600;
    else if(baud == 19200) baud = B19200;
    else if(baud == 38400) baud = B38400;
    else if(baud == 57600) baud = B57600;
    else if(baud == 115200) baud = B115200;
    else if(baud == 230400) baud = B230400;
    else
    {
        std::cout<<"The baud rate can not be specified."<<std::endl;
        return -1;
    }

    const std::string SERIAL_PORT=port; //"/dev/ttyUSB0";//?????????????????????????????????????????????
    unsigned char msg[] = "serial port open...\n";
    unsigned char buf[255];             // ????????????
    int fd;                             // ?????????????????????????????????
    struct termios tio;                 // ????????????????????????
    int baudRate = baud;
    int i;
    int len;
    int ret;
    int size;

    fd = open(SERIAL_PORT.c_str(), O_RDWR);     // ?????????????????????????????????
    if (fd < 0) {
        printf("open error\n");
        return -1;
    }

    tio.c_cflag += CREAD;               // ????????????
    tio.c_cflag += CLOCAL;              // ????????????????????????????????????????????????
    tio.c_cflag += CS8;                 // ??????????????????:8bit
    tio.c_cflag += 0;                   // ?????????????????????:1bit
    tio.c_cflag += PARENB;                   // ????????????:None

    cfsetispeed( &tio, baudRate );
    cfsetospeed( &tio, baudRate );

    cfmakeraw(&tio);                    // RAW?????????ros advertise true

    tcsetattr( fd, TCSANOW, &tio );     // ??????????????????????????????

    ioctl(fd, TCSETS, &tio);            // ????????????????????????????????????

    nmea_pub=nh.advertise<nmea_msgs::Sentence>("nmea_sentence",100,false);
    //time_diff_pub = nh.advertise<std_msgs::Float64>("nmea_time_diff",1);

    //ros::Rate rate(1);
    ros::Time prev_time = ros::Time::now();
    while(ros::ok())
    {
        char buf[300];
        len = read(fd, buf, sizeof(buf));
        /*ros::Time nowtime = ros::Time::now();
        ros::Duration ros_time_diff = nowtime - prev_time;
        double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
        std_msgs::Float64 tdpub;
        tdpub.data = time_diff;
        time_diff_pub.publish(tdpub);
        prev_time = nowtime;*/
        //std::cout<<buf<<std::endl;
        //std::cout<<len<<std::endl;
        if(len <= 0) continue;
        buf[len]='\0';
        publish(buf,len);
        std::cout << "size : " << len << std::endl;
        printf("%s\n\n",buf);
        //rate.sleep();
    }

    close(fd);
    return 0;
}
