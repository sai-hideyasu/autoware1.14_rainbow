#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <nmea_msgs/Sentence.h>
#include <std_msgs/String.h>
#include <string>

namespace {
    ros::Publisher nmea_pub;
    ros::Publisher error_string_pub;
}

void publish(const char buf[], const int bufSize)
{
 	std::string str(buf);
    nmea_msgs::Sentence sentence;
    sentence.header.stamp=ros::Time::now();
    sentence.header.frame_id="gps";
    sentence.sentence=str;
    nmea_pub.publish(sentence);
}

std::string replaceOtherStr(const std::string &replacedStr, const char* from, const char* to) {
    std::string str = replacedStr;
    for(;;)
    {
        int pos = str.find(from);
        std::cout << pos << std::endl;
        if(pos < 0) break;

        str = str.replace(pos, 1, to);
    }

    return str;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"nmea_tcp");
    ros::NodeHandle nh, private_nh("~");

    std::string ip_str;
    private_nh.getParam("ip", ip_str);
    std::cout << ip_str << std::endl;
    int sock = socket(PF_INET, SOCK_STREAM, 0);//ソケットの作成

    //接続先指定用構造体の準備
    struct sockaddr_in server;
    server.sin_family = PF_INET;
    server.sin_port = htons(3001);
    server.sin_addr.s_addr = inet_addr(ip_str.c_str());

    //サーバに接続
    connect(sock, (struct sockaddr *)&server, sizeof(server));

    nmea_pub = nh.advertise<nmea_msgs::Sentence>("nmea_sentence",1);
    error_string_pub = nh.advertise<std_msgs::String>("nmea_sentence_error",1);

    //ros::Rate rate(1);
    while(ros::ok())
    {
        const int BUFSIZE = 400;
        char buf[BUFSIZE];
        int len=read(sock, buf, sizeof(buf));
        if(len <= 0)
        {
            std_msgs::String str;
            str.data = "data size 0";
            error_string_pub.publish(str);
            continue;
        }
        if(len >= BUFSIZE)
        {
            std_msgs::String str;
            str.data = "data size over";
            error_string_pub.publish(str);
            continue;
        }
        buf[len]='\0';
        std::string str = buf;
        std::cout << str << std::endl;
        std::string rep = replaceOtherStr(str, "\"", "'");
        rep = replaceOtherStr(rep, "\r\n", "\0");
        publish(rep.c_str(), rep.size());
        //std::cout << "size : " << len << std::endl;
        printf("%s\n\n",buf);
        //rate.sleep();
    }

    close(sock);
    return 0;
}
