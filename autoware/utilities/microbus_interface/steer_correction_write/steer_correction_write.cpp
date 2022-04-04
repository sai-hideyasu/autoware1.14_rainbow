#include <ros/ros.h>
#include <time.h>
#include <fstream>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <unordered_map>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <autoware_msgs/SteerSubInterfaceCorrection.h>
#include <autoware_msgs/WaypointParam.h>

std::vector<std::string> split(const std::string &string, const char sep)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, sep))
	str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

struct CorrectionInfo
{
	int id;
	int16_t steer_correction_;
};

class SteerCorrectionWrite
{
private:
	const int ID_DISTANCE_TH = 10;//記録された調整情報の経路IDが隣同士の経路IDの数値がこの定数幅異なる場合は、経路が大きく外れたとして処理しない
	const std::string OPERATION_FIELD = "steer_actual_plus_sub";//ステア調整フィールドの名称
	const std::string ID_FIELD = "id";//IDフィールドの名称

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber sub_waypoints_file_name_, sub_steer_correction_, sub_waypoint_param_, sub_write_end_;
	ros::Publisher pub_steer_correction_write_return_, pub_tmp;

	std::string waypoints_file_name_;
	autoware_msgs::SteerSubInterfaceCorrection steer_correction_;
	std::vector<CorrectionInfo> correction_list_;
	bool end_flag_;
	bool waypoints_file_name_change_flag_;
	bool id_distance_flag_;

	void callbackWaypointsFileName(const std_msgs::String::ConstPtr &msg)
	{
		if(waypoints_file_name_ == "")//初回の経路パス以外は扱わない　経路自動切り替え対策
			waypoints_file_name_ = msg->data;
		else
			waypoints_file_name_change_flag_ = true;
	}

	void callbackSteerCorrection(const autoware_msgs::SteerSubInterfaceCorrection::ConstPtr &msg)
	{
		steer_correction_ = *msg;
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		if(steer_correction_.processing == true && waypoints_file_name_change_flag_ == false)
		{
			CorrectionInfo info = {msg->id, steer_correction_.correct_val};
			CorrectionInfo prev_info;
			if(correction_list_.size() != 0) prev_info = correction_list_[correction_list_.size() -1];
			else prev_info.id = -1;
			if(std::abs(info.id - prev_info.id) > ID_DISTANCE_TH && prev_info.id != -1) id_distance_flag_ = true;//記録された調整情報の経路IDが隣同士の経路IDの数値がこの定数幅異なる場合は、経路が大きく外れたとして処理しない
			if(id_distance_flag_ == false) correction_list_.emplace_back(info);
		}
	}

	void endPublish(const std::string message)
	{
		std_msgs::String str;
		str.data = message;
		pub_steer_correction_write_return_.publish(str);
	}

	void callbackWriteEnd(const std_msgs::Bool::ConstPtr &msg)
	{
		end_flag_ = true;
		if(msg->data == false) return;

		if(waypoints_file_name_ == "")
		{
			endPublish("autowareからの経路ファイル読み込みがありません");
			return;
		}

		if(!std::experimental::filesystem::exists(waypoints_file_name_))
		{
			std::stringstream ss;
			ss << "経路ファイルが存在しません\n" << waypoints_file_name_;
			endPublish(ss.str());
			return;
		}

		std::ifstream ifs(waypoints_file_name_, std::ios_base::in);
		if(!ifs.is_open())
		{
			std::stringstream ss;
			ss << "現在の経路ファイルを読み込めません\n" << waypoints_file_name_;
			endPublish(ss.str());
			return;
		}

		//先頭のフィールド名を列挙
		std::vector<std::string> fields;
		size_t field_size;
		{
			std::string field_line;
			std::getline(ifs, field_line);
			fields = split(field_line, ',');
			field_size = fields.size();//idフィールドとsteer_actual_plus_subフィールドの有無でfields.size()が変わってしまうので、この段階のフィールド列数を取得
		}

		//idフィールドとsteer_actual_plus_subフィールドの有無を確認
		bool operation_field_exists = false;//ステア調整フィールドが経路ファイルに存在するか
		bool id_exists = false;//idフィールドが経路ファイルに存在するか
		{
			for(std::string str : fields)
			{
				if(str == OPERATION_FIELD) operation_field_exists = true;
				if(str == ID_FIELD) id_exists = true;
			}
			if(operation_field_exists == false) fields.emplace_back(OPERATION_FIELD);//ステア調整フィールドがない場合は追加
			if(id_exists == false) fields.emplace_back(ID_FIELD);//IDフィールドがない場合は追加
		}

		//経路データをフィールドごとに纏める
		std::vector<std::unordered_map<std::string, std::string>> route_data;//経路データをフィールド毎に纏めたもの
		{
			for(int id_cou=0; !ifs.eof(); id_cou++)
			{
				std::string data_line;
				std::getline(ifs, data_line);
				std::vector<std::string> datas = split(data_line, ',');
				if(datas.size() == 0 && ifs.eof()) break;//データ終了の場合はforを抜ける
				if(field_size != datas.size())//先頭フィールド列と列数が異なる場合は異常ファイル
				{
					std::stringstream ss;
					ss << "経路ファイルのフィールド列数が一致しません\n" << waypoints_file_name_ << "\nid : " << id_cou+1 << "\n先頭フィールド数 : " << field_size << "\n読み込んだフィールド数 : " << datas.size();
					endPublish(ss.str());
					return;
				}

				std::unordered_map<std::string, std::string> map;
				for (size_t i = 0; i < datas.size(); i++)
				{
					if(fields[i] == ID_FIELD) map[ID_FIELD] = std::to_string(id_cou+1);//IDフィールドは自動で連番を入れる
					else map[fields[i]] = datas[i];
				}
				if(operation_field_exists == false) map[OPERATION_FIELD] = "0";//経路ファイルにステア調整フィールドがない場合は0
				if(id_exists == false) map[ID_FIELD] = std::to_string(id_cou+1);//経路ファイルにIDフィールドがない場合は自動で追加
				route_data.emplace_back(map);
			}
		}
		ifs.close();

		//記録された調整情報を整理
		std::vector<int> steer_correction_ave_list;//ID毎に纏めた調整値の平均
		std::vector<bool> steer_correction_processing_list;//ID毎に纏めた調整値があったかのフラグ
		{
			int sum_check = 0;
			for(int cou=0; cou<route_data.size(); cou++)
			{
				std::unordered_map<std::string, std::string> map = route_data[cou];
				int id = std::stoi(map[ID_FIELD]);

				//ID毎に調整値の平均を取る
				int sum = 0;
				int data_count = 0;
				for(int corr_cou=0; corr_cou<correction_list_.size(); corr_cou++)
				{
					CorrectionInfo info = correction_list_[corr_cou];
					if(info.id != id) continue;

					CorrectionInfo prev_info;
					sum += info.steer_correction_;
					data_count++;
				}

				if(sum == 0 || data_count == 0)//調整値の平均が0の場合は調整がされなかったとして未処理扱い
				{
					steer_correction_ave_list.emplace_back(0);
					steer_correction_processing_list.emplace_back(false);
				}
				else
				{
					steer_correction_ave_list.emplace_back(sum / data_count);
					steer_correction_processing_list.emplace_back(true);
				}

				sum_check += sum;
			}

			if(sum_check == 0)
			{
				endPublish(std::string("変更はありませんでした"));
				return;
			}
		}

		//処理が飛ばされたwaypointを補完
		{
			const int SEARCH_WIDTH = 4;//現在見ているIDからこの範囲のIDに処理フラグがない場合は未処理とする
			const int SEARCH_NO = 100000;//未処理判定値
			for(int cou=0; cou<steer_correction_ave_list.size(); cou++)
			{
				if(steer_correction_processing_list[cou] == true) continue;

				//後方の処理IDを探索
				int prev_val = SEARCH_NO;
				int prev_ind;
				for(int prev_cou=1; prev_cou<SEARCH_WIDTH; prev_cou++)
				{
					int ind = cou - prev_cou;
					if(ind < 0) break;
					if(steer_correction_processing_list[ind] == true)
					{
						prev_val = steer_correction_ave_list[ind];
						prev_ind = ind;
						break;
					}
				}

				//前方の処理IDを探索
				int next_val = SEARCH_NO;
				int next_ind;
				for(int prev_cou=1; prev_cou<SEARCH_WIDTH; prev_cou++)
				{
					int ind = cou + prev_cou;
					if(ind >= steer_correction_ave_list.size()) break;
					if(steer_correction_processing_list[ind] == true)
					{
						next_val = steer_correction_ave_list[ind];
						next_ind = ind;
						break;
					}
				}

				//処理IDがSEARCH_WIDTHの範囲内にない場合は未処理として確定
				if(prev_val == SEARCH_NO || next_val == SEARCH_NO) continue;

				//補完処理
				int complement_count = next_ind - prev_ind - 1;tmpPub(std::to_string(complement_count));
				double val_step = (next_val - prev_val) / (complement_count+1);
				for(int i=prev_ind+1, step_cou=1; i<next_ind; i++, step_cou++)
				{
					steer_correction_ave_list[i] = val_step * step_cou + prev_val;
					steer_correction_processing_list[i] = true;
				}
			}
		}

		//元の経路ファイルのバックアップ作成
		{
			std::experimental::filesystem::path path = waypoints_file_name_;
			time_t nowtime = time(NULL);
			tm* date = localtime(&nowtime);
			int year = date->tm_year-100+2000;
			int mou = date->tm_mon+1;
			int day = date->tm_mday;
			int hour = date->tm_hour;
			int min = date->tm_min;
			int sec = date->tm_sec;
			std::stringstream sstime;
			sstime << std::setfill('0') << std::right << std::setw(4) << year << "_" << std::setw(2)<< mou << "_" << std::setw(2) << day << "_" << std::setw(2) << hour << "_" << std::setw(2) << min << "_" << std::setw(2) << sec;
			std::stringstream new_file_name;
			new_file_name << path.parent_path().string() << "/" << path.stem().string() <<"___" <<  sstime.str() << path.extension().string();			
			std::experimental::filesystem::copy(waypoints_file_name_, new_file_name.str());
		}

		//ステア調整情報を書き込む
		{
			std::ofstream ofs(waypoints_file_name_, std::ios_base::out);
			if(!ofs.is_open())
			{
				std::stringstream ss;
				ss << "現在の経路ファイルに書き込めません\n" << waypoints_file_name_;
				endPublish(ss.str());
				return;
			}

			//フィールド列の書き込み　IDフィールドは先頭に書く
			ofs << ID_FIELD;
			for(int cou=0; cou<fields.size(); cou++)
				if(fields[cou] != ID_FIELD) ofs << "," << fields[cou];
			ofs << '\n';

			//各パラメーターを書き込み
			for(int rcou=0; rcou<route_data.size(); rcou++)
			{
				//補正値をステア補正フィールドの数値にプラスする
				std::unordered_map<std::string, std::string> &map = route_data[rcou];
				int steer_curr_plus = steer_correction_ave_list[rcou];
				int steer_curr_orig = std::stoi(map[OPERATION_FIELD]);
				int steer_curr_new = steer_curr_orig + steer_curr_plus;
				map[OPERATION_FIELD] = std::to_string(steer_curr_new);

				ofs << map[ID_FIELD];//経路IDを先頭列に書き込む
				for(int fcou=0; fcou<fields.size(); fcou++)
					if(fields[fcou] != ID_FIELD) ofs << "," << map[fields[fcou]];
				ofs << '\n';
			}
			ofs.flush();
			ofs.close();
		}

		endPublish(std::string("調整完了！"));
	}

	void tmpPub(const std::string str)
	{
		std_msgs::String pub;
		pub.data = str;
		pub_tmp.publish(pub);
	}
public:
	SteerCorrectionWrite(const ros::NodeHandle nh, const ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, end_flag_(false)
		, waypoints_file_name_change_flag_(false)
		, id_distance_flag_(false)
	{
		pub_steer_correction_write_return_ = nh_.advertise<std_msgs::String>("/microbus/steer_correction_write_return", 1);
		pub_tmp = nh_.advertise<std_msgs::String>("/abesi", 1);

		sub_waypoints_file_name_ = nh_.subscribe("/waypoints_file_name", 1, &SteerCorrectionWrite::callbackWaypointsFileName, this);
		sub_steer_correction_ = nh_.subscribe("/microbus/steer_sub_interface_correction", 1, &SteerCorrectionWrite::callbackSteerCorrection, this);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 1, &SteerCorrectionWrite::callbackWaypointParam, this);
		sub_write_end_ = nh_.subscribe("/microbus/steer_sub_interface_correction_write_end", 1, &SteerCorrectionWrite::callbackWriteEnd, this);
	}

	bool getEnd()
	{
		return end_flag_;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "steer_correction_write");
	ros::NodeHandle nh, pnh("~");

	SteerCorrectionWrite scw(nh, pnh);
	ros::Rate rate(100);
	while(ros::ok() && !scw.getEnd())
	{
		ros::spinOnce();
		rate.sleep();
	}

	ros::Rate end_rate(1);
	end_rate.sleep();//publishが消滅しないようにノードの処理を少し待つ
	return 0;
}