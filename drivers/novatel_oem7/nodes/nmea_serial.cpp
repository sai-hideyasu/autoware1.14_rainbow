#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <nmea_msgs/Sentence.h>
#include <std_msgs/String.h>

#include <signal.h>

// 定数

//　受信処理可能フラグ
//  true:デバイスファイルが正常にオープン
//  false:SIGHUP、IGINT、SIGTERMを受信
bool canReceiveData = false;

//　処理可能フラグ
//  true:起動初期値
//  false:SIGINT、SIGTERMを受信
bool canProcess = true;

namespace {
    ros::Publisher nmea_pub, nmea_error_pub, nmea_string_pub, nmea_device_status_pub;
}

static const std::string START_STRING("#");
static const std::string TERMINATE_STRING("\r\n");

//関数プロトタイプ
void publish(const char buf[], const int bufSize);
std::string replaceOtherStr(const std::string &replacedStr, const char* from, const char* to);
void sigHandler(int p_signame);
void setSignal(void);
int setupSerialDevice(std::string port, int baud);
void receiveData(int fd);


void publish(const char buf[], const int bufSize)
{
	std::string str(buf);
    nmea_msgs::Sentence sentence;
    sentence.header.stamp=ros::Time::now();
    sentence.header.frame_id="gps";
    sentence.sentence=str;
    nmea_pub.publish(sentence);
}

std::string replaceOtherStr(const std::string &replacedStr, const char* from, const char* to) 
{
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

void sigHandler(int p_signame)
{
    ROS_WARN("signal catch NO=%d", p_signame);
    switch(p_signame)
    {
      case SIGINT:
      case SIGTERM:
        canProcess = false;  
        // fall throuth
      case SIGHUP:
        canReceiveData = false;    
    }
}

void setSignal(void)
{
    signal(	SIGINT	, sigHandler);
//    signal(	SIGILL	, sigHandler);
//    signal(	SIGABRT	, sigHandler);
//    signal(	SIGFPE	, sigHandler);
//    signal(	SIGSEGV	, sigHandler);
    signal(	SIGTERM	, sigHandler);

/* Historical signals specified by POSIX. */
    signal(	SIGHUP	, sigHandler);
//    signal(	SIGQUIT	, sigHandler);
//    signal(	SIGTRAP	, sigHandler);
//    signal(	SIGKILL	, sigHandler);
//    signal( SIGBUS	, sigHandler);
//    signal(	SIGSYS	, sigHandler);
//    signal(	SIGPIPE	, sigHandler);
//    signal(	SIGALRM	, sigHandler);

/* New(er) POSIX signals (1003.1-2008, 1003.1-2013).  */
//    signal(	SIGURG	, sigHandler);
//    signal(	SIGSTOP	, sigHandler);
//    signal(	SIGTSTP	, sigHandler);
//    signal(	SIGCONT	, sigHandler);
//    signal(	SIGCHLD	, sigHandler);
//    signal(	SIGTTIN	, sigHandler);
//    signal(	SIGTTOU	, sigHandler);
//    signal(	SIGPOLL	, sigHandler);
//    signal(	SIGXCPU	, sigHandler);
//    signal(	SIGXFSZ	, sigHandler);
//    signal(	SIGVTALRM, sigHandler);
//    signal(	SIGPROF	, sigHandler);
//    signal(	SIGUSR1	, sigHandler);
//    signal(	SIGUSR2	, sigHandler);
}

int setupSerialDevice(std::string port, int baud)
{
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

    const std::string SERIAL_PORT=port; //"/dev/ttyUSB0";//デバイス名は適宜変えてください
    int fd;                             // ファイルディスクリプタ
    struct termios tio;                 // シリアル通信設定
    int baudRate = baud;

    std::string command("sudo chmod 666 ");
    command  += SERIAL_PORT;
    while( 0 != system(command.c_str()) )
    {
        ROS_WARN("chmod error\n");
        return -1;
    }

    fd = open(SERIAL_PORT.c_str(), O_RDWR);     // デバイスをオープンする
    if (fd < 0) {
        ROS_WARN("open error\n");
        return -1;
    }

    memset(&tio,0,sizeof(tio));
    tio.c_cflag += CREAD;               // 受信有効
    tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
    tio.c_cflag += CS8;                 // データビット:8bit
    tio.c_cflag += 0;                   // ストップビット:1bit
    tio.c_cflag += PARENB;              // パリティ:None
    tio.c_iflag = IGNPAR;               // パリティエラー無視
    tio.c_lflag = 0;                    // non-canonical
    tio.c_cc[VTIME] = 1;                // read time out 100ms 
    tio.c_cc[VMIN] = 0;                 // １文字受信

    cfsetispeed( &tio, baudRate );      // ボーレート設定
    cfsetospeed( &tio, baudRate );

    tcflush( fd, TCIFLUSH );
    tcsetattr( fd, TCSANOW, &tio );     // デバイスに設定を行う
    ioctl(fd, TCSETS, &tio);            // ポートの設定を有効にする

    canReceiveData = true;
    return fd;
}

void receiveData(int fd)
{
    // receive nmea string data
    std::string nmea_string = "";
    std_msgs::String str_message;

ROS_WARN("step=1");
    //ros::Rate rate(1);
    while(canReceiveData)
    {
        unsigned char buf;
//ROS_WARN("step=2");
        int len = read(fd, &buf, sizeof(buf));
//ROS_WARN("step=3 %x", buf);

        if( len < 0 )
        {
ROS_WARN("step=4");
            // read fatal error
            str_message.data = std::string("read fatal error errno=") + std::to_string(errno);
            nmea_error_pub.publish(str_message);
            break;
        }
        else if( len == 0 )
        {
//ROS_WARN("step=5");
            // read time out
            ros::Time timeout = ros::Time::now();
            str_message.data = std::string("read time out ") + std::to_string(timeout.sec) + "." + std::to_string(timeout.nsec);
            nmea_string_pub.publish(str_message);
            continue;
        }

//ROS_WARN("step=6");
        // 開始文字
        if(START_STRING.front() == buf)
        {
//ROS_WARN("step=7");
            // 受信未完了のデータチェック
            if( nmea_string.length() != 0)
            {
//ROS_WARN("step=8");
                str_message.data = nmea_string;
                nmea_string_pub.publish(str_message);
            }

//ROS_WARN("step=9");
            // 開始データをセット
            nmea_string = START_STRING;
            continue;
        }
        // 開始文字以外 → 開始文字受信前のデータ受信確認
        else if(START_STRING.front() != nmea_string.front())
        {
//ROS_WARN("step=10");
            str_message.data = buf;
            //ROS_WARN("CHAR=%c",buf);
            nmea_string_pub.publish((str_message));

            // 受信中データ破棄
            nmea_string = "";
            continue;
        }
        // 開始文字受信後のデータ受信
        else
        {
//ROS_WARN("step=11");
            nmea_string += buf;

            // 終端文字確認
            std::string::size_type pos = nmea_string.find(TERMINATE_STRING);
            if(pos == std::string::npos)
            {
//ROS_WARN("step=12");
                // terminate data receive no yet. receive continue.
                continue;
            }
            
//ROS_WARN("step=13");
            // 受信完了
            std::string rep = replaceOtherStr(nmea_string, "\"", "'");
            rep = replaceOtherStr(rep, TERMINATE_STRING.c_str(), "\0");
            publish(rep.c_str(), rep.size());
            nmea_string = "";
        }
    }
ROS_WARN("step=14");

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"nmea_serial");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    nmea_pub=nh.advertise<nmea_msgs::Sentence>("nmea_sentence",100,false);
    nmea_error_pub=nh.advertise<std_msgs::String>("nmea_error",1);
    nmea_string_pub=nh.advertise<std_msgs::String>("nmea_string",1);
    nmea_device_status_pub = nh.advertise<std_msgs::String>("/nmea_device_status",1);

    std::string port;
    int baud=0;
    private_nh.getParam("port", port);
    private_nh.getParam("baud", baud);

    setSignal();
    
    while(canProcess)
    {
//ROS_WARN("step=100");
        int fd = setupSerialDevice(port, baud);
        if( 0 <= fd)
        {
//ROS_WARN("step=101");
            std_msgs::String str;
            str.data = "start";
            nmea_device_status_pub.publish(str);

            receiveData(fd);
            close(fd);
            if(canProcess == true && canReceiveData == false)
            {
                std_msgs::String str;
                str.data = "error_canReceiveData";
                nmea_device_status_pub.publish(str);
            }
        }
	// 50ms sleep
        usleep(50000);
    }

//ROS_WARN("step=9999");
    return 0;
}
