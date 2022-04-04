#include <ros/ros.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <nmea_msgs/Sentence.h>
#include <autoware_msgs/NmeaArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

namespace {
    ros::Publisher nmea_pub, nmea_error_pub, nmea_string_pub;//, time_diff_pub;
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
        //std::cout << pos << std::endl;
        if(pos < 0) break;

        str = str.replace(pos, 1, to);
    }

    return str;
}

int main(int argc, char** argv)
{
    // initialize node
    ros::init(argc,argv,"nmea_tcp");
    ros::NodeHandle nh, private_nh("~");
    nmea_pub=nh.advertise<nmea_msgs::Sentence>("nmea_sentence",1);
    nmea_error_pub=nh.advertise<std_msgs::String>("nmea_error",1);
    nmea_string_pub=nh.advertise<std_msgs::String>("nmea_string",1);
    //time_diff_pub = nh.advertise<std_msgs::Float64>("nmea_time_diff",1);

    // setup server infomation
    std::string ip_str;
    int port;
    private_nh.getParam("ip", ip_str);
    private_nh.getParam("port", port);

// debug
//ip_str = "127.0.0.1";

    // connect server PortNo=3001
    boost::asio::io_service io;
    boost::asio::ip::tcp::socket sock(io);
    boost::asio::ip::tcp::endpoint endpoint = boost::asio::ip::tcp::endpoint{boost::asio::ip::address::from_string(ip_str), 3001};
    sock.connect(endpoint);

//    ros::Rate rate(1);
    //ros::Time prev_time = ros::Time::now();
    while(ros::ok())
    {
        //read until "\r\n"
        boost::system::error_code error;
        boost::asio::streambuf readbuf;
        boost::asio::read_until(sock,readbuf,"\r\n",error);
        /*ros::Time nowtime = ros::Time::now();
        ros::Duration ros_time_diff = nowtime - prev_time;
        double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
        std_msgs::Float64 tdpub;
        tdpub.data = time_diff;
        time_diff_pub.publish(tdpub);
        prev_time = nowtime;*/
        //boost::asio::read_until(sock,readbuf,"\n",error);

        std::string all_str = boost::asio::buffer_cast<const char*>(readbuf.data());
        std::vector<std::string> nmea_array;
        boost::algorithm::split(nmea_array, all_str, boost::is_any_of("\n"), boost::algorithm::token_compress_on);//boost::is_any_of("\n"));

        for(std::string nmea_parts : nmea_array)
        {
            std::cout << nmea_parts << std::endl;
        }

        for(std::string nmea_parts : nmea_array)
        {
            std::string::iterator begin = nmea_parts.begin();
            std::string::iterator end = nmea_parts.end();
            std::cout << *(end-1) << std::endl;
            if(*begin == '#' && *(end-1) == '\r')
            {//OK
                publish(nmea_parts.c_str(), nmea_parts.size());
                break;
            }
            else
            {
                std_msgs::String str_message;
                str_message.data = nmea_parts;
                nmea_string_pub.publish(str_message);
            }
        }

        // check read error
        /*if(error && (error != boost::asio::error::eof))
        {
            // Discard data
            std_msgs::String str;
            str.data = "error,eof";
            nmea_error_pub.publish(str);
            std::cout << "error,eof" << std::endl;
            continue;
        }else{
            // Confirm that the data starts with "#""
            std::string str = boost::asio::buffer_cast<const char *>(readbuf.data());
            std_msgs::String str_message;
            str_message.data = str;
            nmea_string_pub.publish(str_message);
            if( '#' != str.at(0))
            {
                // Discard data
                std_msgs::String str_error;
                str_error.data = "error,no #";
                nmea_error_pub.publish(str_error);
                std::cout << "error,not #" << std::endl;;
                std::cout << str_message << std::endl;
                continue;
            }
            // TODO Check CRC

            // publish data
            std::string rep = replaceOtherStr(str, "\"", "'");
            rep = replaceOtherStr(rep, "\r\n", "\0");
            publish(rep.c_str(), rep.size());
        }*/
//        rate.sleep();
    }
    return 0;
}

