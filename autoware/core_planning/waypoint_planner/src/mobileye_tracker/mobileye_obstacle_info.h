//同じmobileye IDのデータを纏めて管轄するクラス
class MobileyeObstacleInfo
{
public:
	const static size_t FORREGROUND_VELOCITY_LIST_SIZE = 10; //!< 前方車両速度listの最大サイズ
private:
	std::vector<autoware_msgs::TransformMobileyeObstacle> obs_list_; //!< 前方車両のリスト
	double obs_velocity_ave_; //!< 前方車両絶対速度の平均
	double obs_relative_vel_ave_; //!< 前方車両相対速度の平均
	double obs_relative_acc_ave_; //!< 前方車両相対加速度の平均
	double obs_acc_ave_; //!< 前方車両絶対加速度の平均
	double obs_relative_pos_x_ave_; //!< 前方車両相対x位置の平均

	double mathVelAve()
	{
		double vel_ave_ = 0;
		obs_relative_vel_ave_ = 0;
		obs_relative_acc_ave_ = 0;
		obs_relative_pos_x_ave_ = 0;

		for(const autoware_msgs::TransformMobileyeObstacle &obs : obs_list_)
		{
			vel_ave_ += obs.velocity_mps;
			obs_relative_vel_ave_ += obs.orig_data.obstacle_rel_vel_x;
			obs_relative_acc_ave_ += obs.orig_data.object_accel_x;
			obs_relative_pos_x_ave_ += obs.orig_data.obstacle_pos_x;
		}
		obs_relative_vel_ave_ /= obs_list_.size();
		obs_relative_acc_ave_ /= obs_list_.size();
		obs_relative_pos_x_ave_ /= obs_list_.size();
		return vel_ave_ / obs_list_.size();
	}
public:
	MobileyeObstacleInfo()
		: obs_velocity_ave_(0)
		, obs_acc_ave_(0)
		, obs_relative_vel_ave_(0)
		, obs_relative_acc_ave_(0)
		, obs_relative_pos_x_ave_(0)
	{
	}

	void push(const autoware_msgs::TransformMobileyeObstacle &obs)
	{
		obs_list_.push_back(obs);
		if(obs_list_.size() > FORREGROUND_VELOCITY_LIST_SIZE)
			obs_list_.erase(obs_list_.begin());

		double vel = mathVelAve();

		if(obs_list_.size() >= 2)
		{
			ros::Duration ros_time_diff =
				obs_list_[obs_list_.size()-1].header.stamp - obs_list_[obs_list_.size()-2].header.stamp;
			double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
			obs_acc_ave_ = (vel - obs_velocity_ave_) / time_diff;
		}
		else
		{
			obs_acc_ave_ = 0;
		}
		obs_velocity_ave_ = vel;
	}

	size_t getSize() const
	{
		return obs_list_.size();
	}

	autoware_msgs::TransformMobileyeObstacle getObs(const int index) const
	{
		return obs_list_[index];
	}

	autoware_msgs::TransformMobileyeObstacle getLatestObs() const
	{
		return obs_list_[obs_list_.size()-1];
	}

	double getObsAgeTime(const int index) const
	{
		double sum = 0;
		for(int i=1; i<=index; i++)
		{
			ros::Duration rostimediff = obs_list_[i].header.stamp - obs_list_[i-1].header.stamp;
			sum += rostimediff.sec +rostimediff.nsec * 1E-9;
		}
		return sum;
	}

	double getLatestObsAgeTime() const
	{
		return getObsAgeTime(obs_list_.size()-1);
	}

	double getLatestVelocityMps() const
	{
		return obs_list_[obs_list_.size()-1].velocity_mps;
	}

	double getVelocityMpsAve() const
	{
		return obs_velocity_ave_;
	}

	double getVelocityMpsLeastSquares() const
	{
		if(obs_list_.size() == 0) return 0;

		double time_ave = 0, vel_ave = 0;
		ros::Time rosfirsttime = obs_list_[0].header.stamp;

		for(const autoware_msgs::TransformMobileyeObstacle &obs : obs_list_)
		{
			ros::Duration rosobstime = obs.header.stamp - rosfirsttime;
			double obstime = rosobstime.sec + rosobstime.nsec * 1E-9;
			time_ave += obstime;
			vel_ave += obs.velocity_mps;
		}
		time_ave /= obs_list_.size();
		vel_ave /= obs_list_.size();

		double bunsi=0, bunbo=0;
		for(const autoware_msgs::TransformMobileyeObstacle &obs : obs_list_)
		{
			ros::Duration rosobstime = obs.header.stamp - rosfirsttime;
			double obstime = rosobstime.sec + rosobstime.nsec * 1E-9;
			double x = obstime - time_ave;
			double y = obs.velocity_mps - vel_ave;
			bunsi += x * y;
			bunbo += x * x;
		}

		return bunsi/bunbo;
	}

	double getAccAve() const
	{
		return obs_acc_ave_;
	}

	double getObsRelativeVelMpsAve() const
	{
		return obs_relative_vel_ave_;
	}

	double getObsRelativeAccAve() const
	{
		return obs_relative_acc_ave_;
	}

	double getRelativePoseXAve() const
	{
		return obs_relative_pos_x_ave_;
	}

	void clear()
	{
		obs_list_.clear();
		obs_velocity_ave_ = 0;
		obs_relative_vel_ave_ = 0;
		obs_relative_acc_ave_ = 0; 
		obs_acc_ave_ = 0;
		obs_relative_pos_x_ave_ = 0;
	}
};
