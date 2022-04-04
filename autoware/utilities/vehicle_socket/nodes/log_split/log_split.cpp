#include <ros/ros.h>
#include <unordered_map>
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

class LogSplit
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
public:
	LogSplit(const ros::NodeHandle nh, const ros::NodeHandle pnh, const std::string read_log_path, const std::string write_log_path,
		const double first_hour, const double first_min, const double end_hour, const double end_min)
		: nh_(nh)
		, pnh_(pnh)
	{
		std::ifstream ifs(read_log_path, std::ios_base::in);
		if(!ifs.is_open())
		{
			std::cout << "readファイルが開けない" << std::endl;
			return;
		}

		std::ofstream ofs(write_log_path, std::ios_base::out);
		if(!ofs.is_open())
		{
			std::cout << "writeファイルが開けない" << std::endl;
			return;
		}

		std::vector<std::string> fields;
		{
			std::string line;
			std::getline(ifs, line);
			ofs << line << std::endl;
			fields = split(line, ',');
		}
		while(!ifs.eof())
		//for(int a=0;a<1000;a++)
		{
			std::string line;
			std::getline(ifs, line);
			std::vector<std::string> colums = split(line, ',');
			//std::cout << line.size() << "," << colums.size() << std::endl;
			if(fields.size() != colums.size()) continue;

			std::unordered_map<std::string, std::string> map;
			for(int i=0; i<fields.size(); i++)
			{
				map[fields[i]] = colums[i];
			}
			//std::cout << map["time"] << std::endl;

			std::string time_str = map["time"];
			std::vector<std::string> time_split1 = split(time_str, '_');
			std::vector<std::string> time_split2 = split(time_split1[1], '-');
			double hour = std::atof(time_split2[0].c_str());
			double minutes = std::atof(time_split2[1].c_str());
			//std::cout << hour << "," << minutes << std::endl;
			double select_time = hour*60*60 + minutes*60;
			double first_time = first_hour*60*60 + first_min*60;
			double end_time = end_hour*60*60 + end_min*60;

			if(select_time >= first_time && select_time <= end_time)
			{
				ofs << line << std::endl;
				//std::cout << map["time"] << std::endl;
			}
		}
		ifs.close();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "log_split");
	ros::NodeHandle nh, pnh("~");

	std::string read_log_path = pnh.param<std::string>("read_log_path", "");
	std::string write_log_path = pnh.param<std::string>("write_log_path", "/tmp/log_split.csv");
	double first_hour = pnh.param<double>("first_hour", 0);
	double first_min = pnh.param<double>("first_min", 0);
	double end_hour = pnh.param<double>("end_hour", 0);
	double end_min = pnh.param<double>("end_min", 0);

	LogSplit log_split(nh, pnh, read_log_path, write_log_path, first_hour, first_min, end_hour, end_min);
	return 0;
}