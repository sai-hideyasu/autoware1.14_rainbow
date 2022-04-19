//同じesr IDのデータを纏めて管轄するクラス
class EsrObstacleInfo
{
public:
	const static size_t FORREGROUND_VELOCITY_LIST_SIZE = 20; //!< 前方車両速度listの最大サイズ
private:
	std::vector<autoware_msgs::TransformEsrObstacle> obs_list_; //!< 障害物のリスト
	double obs_velocity_ave_; //!< 前方車両絶対速度の平均
	double obs_relative_vel_ave_; //!< 前方車両相対速度の平均
	double obs_relative_acc_ave_;  //!< 前方車両相対加速度の平均
	double obs_acc_ave_; //!< 前方車両絶対加速度の平均
	double obs_relative_pos_x_ave_; //!< 前方車両相対x位置の平均
public:
	EsrObstacleInfo()
		: obs_velocity_ave_(0)
		, obs_acc_ave_(0)
		, obs_relative_vel_ave_(0)
		, obs_relative_acc_ave_(0)
		, obs_relative_pos_x_ave_(0)
	{
	}

	double mathVelAve()
	{
		double vel_ave_ = 0;
		obs_relative_vel_ave_ = 0;
		obs_relative_acc_ave_ = 0;
		obs_relative_pos_x_ave_ = 0;

		for(const autoware_msgs::TransformEsrObstacle &obs : obs_list_)
		{
			double vx = obs.velocity_mps.linear.x;
			double vy = obs.velocity_mps.linear.y;
			vel_ave_ += std::sqrt(vx*vx + vy*vy);
			vx = obs.orig_data.twist.twist.linear.x;
			vy = obs.orig_data.twist.twist.linear.y;
			obs_relative_vel_ave_ += std::sqrt(vx*vx + vy*vy);
			vx = obs.orig_data.accel.accel.linear.x;
			vy = obs.orig_data.accel.accel.linear.y;
			obs_relative_acc_ave_ += std::sqrt(vx*vx + vy*vy);//obs.orig_data.object_accel_x;
			obs_relative_pos_x_ave_ += obs.orig_data.twist.twist.linear.x;
		}
		obs_relative_vel_ave_ /= obs_list_.size();
		obs_relative_acc_ave_ /= obs_list_.size();
		obs_relative_pos_x_ave_ /= obs_list_.size();
		return vel_ave_ / obs_list_.size();
	}

	void push(const autoware_msgs::TransformEsrObstacle &obs)
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

	autoware_msgs::TransformEsrObstacle getObs(const int index) const
	{
		return obs_list_[index];
	}

	autoware_msgs::TransformEsrObstacle getLatestObs() const
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

	double getVelocityAve() const
	{
		return obs_velocity_ave_;
	}

	double getAccAve() const
	{
		return obs_acc_ave_;
	}

	double getObsRelativeVelAve() const
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

	geometry_msgs::Point getRelativePoseAve() const
	{
		geometry_msgs::Point ret;
		ret.x = ret.y = ret.z = 0;
		for(const autoware_msgs::TransformEsrObstacle &obs : obs_list_)
		{
			ret.x += obs.orig_data.pose.pose.position.x;
			ret.y += obs.orig_data.pose.pose.position.y;
			ret.z += obs.orig_data.pose.pose.position.z;
		}
		ret.x /= obs_list_.size();
		ret.y /= obs_list_.size();
		ret.z /= obs_list_.size();
		return ret;
	}

	geometry_msgs::Point getMapPoseAve() const
	{
		geometry_msgs::Point ret;
		ret.x = ret.y = ret.z = 0;
		for(const autoware_msgs::TransformEsrObstacle &obs : obs_list_)
		{
			ret.x += obs.map_pose.position.x;
			ret.y += obs.map_pose.position.y;
			ret.z += obs.map_pose.position.z;
		}
		ret.x /= obs_list_.size();
		ret.y /= obs_list_.size();
		ret.z /= obs_list_.size();
		return ret;
	}

	geometry_msgs::Point getRelativePoseCov() const
	{
		geometry_msgs::Point po_ave = getRelativePoseAve();
		double sumx = 0, sumy=0, sumz=0;
		for(const autoware_msgs::TransformEsrObstacle &obs : obs_list_)
		{
			sumx +=  std::pow(obs.orig_data.pose.pose.position.x - po_ave.x, 2);
			sumy +=  std::pow(obs.orig_data.pose.pose.position.y - po_ave.y, 2);
			sumz +=  std::pow(obs.orig_data.pose.pose.position.z - po_ave.z, 2);
		}
		geometry_msgs::Point ret;
		ret.x = std::sqrt(sumx / obs_list_.size());
		ret.y = std::sqrt(sumy / obs_list_.size());
		ret.z = std::sqrt(sumz / obs_list_.size());
		return ret;
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