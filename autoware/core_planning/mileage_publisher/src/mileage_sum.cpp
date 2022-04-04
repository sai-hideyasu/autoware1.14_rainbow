#include <ros/ros.h>
#include <dirent.h>
#include <fstream>

std::vector<std::string> split(const std::string &string, const char sep)
{
	std::vector<std::string> str_vec_ptr;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, sep))
		str_vec_ptr.push_back(token);

	return str_vec_ptr;
}

bool timeCheck(const int year_select, int mon_select,
	const int year_lower_limit, const int year_upper_limit, const int mon_lower_limit, const int mon_upper_limit)
{
	if(year_lower_limit <= 0 || year_upper_limit <= 0 || mon_lower_limit <= 0 || mon_upper_limit <= 0)
		return true;

	int year = year_lower_limit, mon = mon_lower_limit;
	int year_cou;
	for(year_cou=0; year_cou<100; year_cou++)
	{
		while(mon <= 12)
		{
			if(year_select == year && mon_select == mon)
				return true;
			mon++;
		}
		year++;
		mon = 0;
	}

	return false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mileage_sum");
	ros::NodeHandle nh, pnh("~");

	std::string work_folder = pnh.param<std::string>("folder", "");
	int year_upper_limit = pnh.param<int>("year_upper_limit", 0);
	int year_lower_limit = pnh.param<int>("year_lower_limit", 0);
	int mon_upper_limit = pnh.param<int>("mon_upper_limit", 0);
	int mon_lower_limit = pnh.param<int>("mon_lower_limit", 0);

	DIR *dp;
	dp = opendir(work_folder.c_str());
	if (dp==NULL)
	{
		std::cout << "フォルダが無い" << std::endl;
		return 0;
	}

    dirent* entry = readdir(dp);
	double mileage_sum = 0;
    while (entry != NULL){
        if (entry != NULL) {
			if(entry->d_type == DT_REG)
			{
				std::vector<std::string> file_name_parts = split(std::string(entry->d_name), '-');
				if(file_name_parts.size() != 7)
				{
					std::cout << "ファイル名規則が合わない：" << entry->d_name << std::endl;
					continue;
				}
				int year = std::atoi(split(file_name_parts[0], '_')[1].c_str());
				int mon = std::atoi(file_name_parts[1].c_str());
				int day = std::atoi(file_name_parts[2].c_str());
				//std::cout << year << "," << mon << "," << day << std::endl;
				bool time_check = timeCheck(year, mon, year_lower_limit, year_upper_limit, mon_lower_limit, mon_upper_limit);
				if(time_check == false)
				{
					std::cout << "探索範囲外です：" << entry->d_name << std::endl;
					continue;
				}
  
				std::ifstream ifs(work_folder + "/" + entry->d_name, std::ios_base::in);
				if(!ifs.is_open())
				{
					std::cout << "集計失敗：" << entry->d_name << std::endl;
					return 0;
				}
				std::string filed_line;
				std::getline(ifs, filed_line);
				std::vector<std::string> fields = split(filed_line, ',');
				if(fields.size() != 1)
				{
					std::cout << "フィールド数が不正：" << entry->d_name << ",field size:" << fields.size() << std::endl;
					ifs.close();
					continue;
				}
				int val_ind = -1;
				for(int i=0; i<fields.size() ;i++)
					if(fields[0] == "current_pose") {val_ind = i; break;}
				if(val_ind == -1)
				{
					std::cout << "current_poseフィールドがない：" << entry->d_name << std::endl;
					ifs.close();
					continue;
				}

				std::string val_line;
				std::getline(ifs, val_line);
				std::vector<std::string> vals = split(val_line, ',');
				if(vals.size() != fields.size())
				{
					std::cout << "フィールド列と値列の数が合わない：" << entry->d_name << std::endl;
					ifs.close();
					continue;
				}
				mileage_sum += std::atof(vals[val_ind].c_str());
				if(vals[val_ind] != "0")
					std::cout << "集計：" << work_folder.c_str() << entry->d_name << ",走行距離：" << std::atof(vals[val_ind].c_str()) << std::endl;

				ifs.close();
			}
        }
        entry = readdir(dp);
    }

	std::cout << "mileage sum : " << std::fixed << std::setprecision(6) << mileage_sum << std::endl;
	return 0;
}