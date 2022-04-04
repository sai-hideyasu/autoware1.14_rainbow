#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>
#include <autoware_msgs/UltrasoundDataList.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

struct BitInfo
{
	bool bit_;
	double read_time_;
};

//pinのON,OFFから周波数を作成するクラス
class UltrasoundDeviceInfo
{
public:
	const size_t BIT_HISTORY_SIZE = 15; //bit履歴の最大数
	const double NOT_DETECTION_TIME = 10000.0;//物体検出がされなかった場合の時間指定
	const double NOT_DETECTION_HZ = 1.0 / NOT_DETECTION_TIME;//物体検出がされなかった場合の時間周波数
	const static double NOT_OBS_DISTANCE = 10000.0;//障害物が存在しない場合の指定距離

	//車両の超音波センサーの
	const int PIN_POS_FRONT_LEFT = 7;
	const int PIN_POS_FRONT_CENTER = 8;
	const int PIN_POS_FRONT_RIGHT = 9;
	const int PIN_POS_BACK_LEFT = 4;
	const int PIN_POS_BACK_CENTER = 5;
	const int PIN_POS_BACK_RIGHT = 6;
	const int PIN_POS_LEFT_SHALLOWBACK = 3;
	const int PIN_POS_LEFT_BACK = 2;
	const int PIN_POS_RIGHT_FRONT = 10;
	const int PIN_POS_RIGHT_SHALLOWFRONT = 11;
	const int PIN_POS_RIGHT_BACK = 14;
	const int PIN_POS_RIGHT_SHALLOWBACK = 15;
private:
	unsigned int pinID_;//該当するarduino PIN
	geometry_msgs::Pose position_;//この超音波デバイスのbase_link上の配置
	std::string tf_name_;//このデバイスのtf名
	uint32_t pos_flag_;//車両上の位置フラグ
	double max_range_;//このデバイスの最大射程
	std::vector<BitInfo> bit_history_;//前のビット読み込みの状態 trueなら該当pinビットが１
	int bit_interval_;//２つのonビット間隔 この数値が0の場合はonビットが１つだけ
	double conversion_time_;//現在の応答時間
	double conversion_hz_;//conversion_time_を周波数にしたもの
public:
	UltrasoundDeviceInfo(const unsigned int pinID, const geometry_msgs::Pose position, const double max_range,
		const std::string tf_name, const uint32_t pos_flag)
		: pinID_(pinID)
		, position_(position)
		, max_range_(max_range)
		, tf_name_(tf_name)
		, pos_flag_(pos_flag)
		, conversion_time_(NOT_DETECTION_TIME)
		, conversion_hz_(NOT_DETECTION_HZ)
		, bit_interval_(-1)
	{
		if(tf_name.c_str()[0] != '/')
			tf_name_ = std::string("/") + tf_name_;
	}

	void printInfo()
	{
		std::cout << "pinID," << pinID_ << std::endl;
		std::cout << "x," << position_.position.x << std::endl;
		std::cout << "y," << position_.position.y << std::endl;
		std::cout << "z," << position_.position.z << std::endl;
		tf::Quaternion qua(position_.orientation.x, position_.orientation.y, position_.orientation.z, position_.orientation.w);
		double roll, pitch, yaw;
		tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);
		std::cout << "yaw," << yaw << std::endl;
		std::cout << "roll," << roll << std::endl;
		std::cout << "pitch," << pitch << std::endl;
		std::cout << "max_range," << max_range_ << std::endl;
		std::cout << "tf_name," << tf_name_ << std::endl;
		std::cout << "pos_flag," << pos_flag_ << std::endl << std::endl;
	}

	unsigned int getPinID() const {return pinID_;}
	geometry_msgs::Pose getPosition() const {return position_;}
	std::string getTfName() const {return tf_name_;}
	double getConversionTime() const {return conversion_time_;}
	double getConversionHz() const {return conversion_hz_;}
	uint32_t getPosFlag() const {return pos_flag_;}

	std::vector<uint8_t> getBitHistory() const
	{
		std::vector<uint8_t> ret;
		for(int i=bit_history_.size()-1; i>=0; i--)
		{
			if(bit_history_[i].bit_ == true) ret.push_back(1);
			else ret.push_back(0);
		}
		return ret;
	}

	//pinのON,OFFを受け取って周波数を作成する
	void setBit(const bool setbit, double time)
	{
		//新たしいON,OFF状態を履歴に追加
		BitInfo info = {setbit, time};
		bit_history_.push_back(info);
		if(bit_history_.size() > BIT_HISTORY_SIZE)
			bit_history_.erase(bit_history_.begin());

		//履歴を新しい順に捜査して、最初のON状態のビットの場所を設定
		int start_ind = -1;
		double start_time;
		for(int cou=bit_history_.size()-1; cou>=0; cou--)
		{
			if(bit_history_[cou].bit_ == true)
			{
				start_ind = cou;
				start_time = bit_history_[cou].read_time_;
				break;
			}
		}
		if(start_ind == -1)
		{
			conversion_time_ = NOT_DETECTION_TIME;
			conversion_hz_ = NOT_DETECTION_HZ;
			bit_interval_ = -1;
			return;
		}

		//更に先を捜査して、次のON状態のビットの場所を設定
		int end_ind = -1;
		double end_time;
		int on_counter = 1;//ONフラグビットの数
		int off_counter = 0;//OFFフラグビットの数
		for(int cou=start_ind-1; cou>=0; cou--)
		{
			if(bit_history_[cou].bit_ == true)
			{
				on_counter++;
				if(on_counter == 3 && off_counter == 0)//近接判定
				{
					end_ind = start_ind - 1;
					end_time = bit_history_[end_ind].read_time_;
					break;
				}
				if(!(on_counter == 2 && off_counter == 0))
				{
					end_ind = cou;
					end_time = bit_history_[cou].read_time_;
					break;					
				}
			}
			else off_counter++;
		}
		if(end_ind == -1)
		{
			conversion_time_ = NOT_DETECTION_TIME;
			conversion_hz_ = NOT_DETECTION_HZ;
			bit_interval_ = 0;
			return;
		}

		conversion_time_ = start_time - end_time;
		conversion_hz_ = 1.0 / conversion_time_;
		bit_interval_ = start_ind - end_ind;
	}

	int getBitIntarval() const
	{
		return bit_interval_;
	}

	double getDistance() const
	{
		if(bit_interval_ == -1) return NOT_OBS_DISTANCE;
		else if(bit_interval_ == 1) return max_range_ * (30.0/60.0);
		else if(bit_interval_ == 2) return max_range_ * (35.0/60.0);
		else if(bit_interval_ == 3) return max_range_ * (40.0/60.0);
		else if(bit_interval_ == 4) return max_range_ * (45.0/60.0);
		else if(bit_interval_ == 5) return max_range_ * (50.0/60.0);
		else if(bit_interval_ == 6) return max_range_ * (55.0/60.0);
		else if(bit_interval_ == 0) return max_range_ * (60.0/60.0);
		else return max_range_ * (60.0/60.0);
	}
};

class ArduinoUltrasound
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Publisher pub_ultrasound_data_list_;//超音波センサーからのビット列をpublish
	ros::Publisher pub_ultrasound_num_;//超音波センサーの数をpublish

	ros::Subscriber sub_bit_list_;//arduinoからの全チャンネルの周波数情報をbitで表現したもの
	ros::Subscriber sub_distance_array_;//センサーから検出された距離の一覧をpublish

	ros::Timer timer_;

	tf::TransformBroadcaster broadcaster_;

	std::vector<UltrasoundDeviceInfo> data_list_;//全超音波情報

	void callbackBitList(const std_msgs::UInt32::ConstPtr &msg)
	{
		ros::Time ros_nowtime = ros::Time::now();
		double nowtime = ros_nowtime.sec + ros_nowtime.nsec * 1E-9;

		//pin毎のbit状態をセット
		autoware_msgs::UltrasoundDataList pubdata;
		pubdata.header.stamp = ros::Time::now();
		for(int cou=0; cou<data_list_.size(); cou++)
		{
			UltrasoundDeviceInfo &info = data_list_[cou];
			int pinID = info.getPinID();
			int bit = (msg->data >> (pinID - 1)) & 0x1;
			if(bit > 0) info.setBit(true, nowtime);
			else info.setBit(false, nowtime);

			//if(pinID != 8) continue;
			autoware_msgs::UltrasoundData data;
			data.pin_id = info.getPinID();
			data.conversion_time = info.getConversionTime();
			data.bit_history = info.getBitHistory();
			data.bit_interval = info.getBitIntarval();
			data.object_distance = info.getDistance();
			data.position_flag = info.getPosFlag();
			pubdata.list.push_back(data);
		}
		pub_ultrasound_data_list_.publish(pubdata);
	}

	void callbackTimer(const ros::TimerEvent &e)
	{
		ros::Time nowtime = e.current_real;

		//autoware_msgs::UltrasoundDataList pubdata;
		//pubdata.header.stamp = ros::Time::now();
		for(UltrasoundDeviceInfo &info : data_list_)
		{
			/*autoware_msgs::UltrasoundData data;
			data.pin_id = info.getPinID();
			data.conversion_time = info.getConversionTime();
			data.bit_history = info.getBitHistory();
			data.bit_interval = info.getBitIntarval();
			pubdata.list.push_back(data);*/

			double distance = info.getDistance();
			if(distance != UltrasoundDeviceInfo::NOT_OBS_DISTANCE)
			{
				tf::Transform transform;
				transform.setOrigin(tf::Vector3(distance, 0, 0));
				transform.setRotation(tf::Quaternion::getIdentity());
				broadcaster_.sendTransform(tf::StampedTransform(transform, nowtime, info.getTfName(), info.getTfName()+"_obs"));
			}
		}
		//pub_ultrasound_data_list_.publish(pubdata);
	}
public:
	ArduinoUltrasound(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		int dev_cou = pnh_.param<int>("dev_count", 0);

		//デバイス番号０〜dev_countまでを確保
		for(int cou=1; cou<=dev_cou; cou++)
		{
			unsigned int dev_num = pnh_.param<int>(std::string("dev") + std::to_string(cou), 0);

			geometry_msgs::Pose position;
			position.position.x = pnh_.param<double>(std::string("posx") + std::to_string(cou), 0.0);
			position.position.y = pnh_.param<double>(std::string("posy") + std::to_string(cou), 0.0);
			position.position.z = pnh_.param<double>(std::string("posz") + std::to_string(cou), 0.0);
			double yaw = pnh_.param<double>(std::string("yaw") + std::to_string(cou), 0.0);
			tf::Quaternion qua = tf::createQuaternionFromYaw(yaw);
			position.orientation.x = qua.x();
			position.orientation.y = qua.y();
			position.orientation.z = qua.z();
			position.orientation.w = qua.w();

			double max_range = pnh_.param<double>(std::string("max_range") + std::to_string(cou), 0.6);
			std::string tf_name = pnh_.param<std::string>(std::string("tf_name") + std::to_string(cou), "/nazo");
			uint8_t pos_num = (uint8_t)pnh_.param<int>(std::string("pos_flag") + std::to_string(cou), 0);
			uint32_t pos_flag = 0;
			if(pos_num == 1) pos_flag = autoware_msgs::UltrasoundData::POS_FRONT_LEFT;
			else if(pos_num == 2) pos_flag = autoware_msgs::UltrasoundData::POS_FRONT_CENTER;
			else if(pos_num == 3) pos_flag = autoware_msgs::UltrasoundData::POS_FRONT_RIGHT;
			else if(pos_num == 4) pos_flag = autoware_msgs::UltrasoundData::POS_BACK_LEFT;
			else if(pos_num == 5) pos_flag = autoware_msgs::UltrasoundData::POS_BACK_CENTER;
			else if(pos_num == 6) pos_flag = autoware_msgs::UltrasoundData::POS_BACK_RIGHT;
			else if(pos_num == 7) pos_flag = autoware_msgs::UltrasoundData::POS_LEFT_FRONT;
			else if(pos_num == 8) pos_flag = autoware_msgs::UltrasoundData::POS_LEFT_SHALLOWFRONT;
			else if(pos_num == 9) pos_flag = autoware_msgs::UltrasoundData::POS_LEFT_SHALLOWBACK;
			else if(pos_num == 10) pos_flag = autoware_msgs::UltrasoundData::POS_LEFT_BACK;
			else if(pos_num == 11) pos_flag = autoware_msgs::UltrasoundData::POS_RIGHT_FRONT;
			else if(pos_num == 12) pos_flag = autoware_msgs::UltrasoundData::POS_RIGHT_SHALLOWFRONT;
			else if(pos_num == 13) pos_flag = autoware_msgs::UltrasoundData::POS_RIGHT_SHALLOWBACK;
			else if(pos_num == 14) pos_flag = autoware_msgs::UltrasoundData::POS_RIGHT_BACK;
			UltrasoundDeviceInfo info(dev_num, position, max_range, tf_name, pos_flag);
			info.printInfo();
			data_list_.push_back(info);
		}

		pub_ultrasound_data_list_ = nh_.advertise<autoware_msgs::UltrasoundDataList>("/ultrasound_data_list", 1, false);
		pub_ultrasound_num_ = nh_.advertise<std_msgs::UInt32>("/arduino_ultrasound/device_num", 1, true);
		sub_bit_list_ = nh_.subscribe("/arduino_ultrasound/bit_list", 1, &ArduinoUltrasound::callbackBitList, this);//arduinoからの各超音波デバイス応答を受け取る
		timer_ = nh_.createTimer(ros::Duration(0.05), &ArduinoUltrasound::callbackTimer, this);

		std_msgs::UInt32 dev_pub;
		dev_pub.data = dev_cou;
		pub_ultrasound_num_.publish(dev_pub);
	}

	size_t getDevCount() const
	{
		return data_list_.size();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arduino_ultrasound");
	ros::NodeHandle nh, pnh("~");

	ArduinoUltrasound au(nh, pnh);
	ros::spin();
	return 0;
}