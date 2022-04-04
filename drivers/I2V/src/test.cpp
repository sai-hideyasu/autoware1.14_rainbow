#include "reader/reader.h"
#include "sender/sender.h"
#include "service_load_signal_info/service_load_signal_info.h"
#include "car_signal_info/car_signal_info.h"
#include "walker_signal_info/walker_signal_info.h"
#include "other.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
const int TRAFFIC_LIGHT_RED = 0;
const int TRAFFIC_LIGHT_GREEN = 1;
const int TRAFFIC_LIGHT_UNKNOWN = 2;

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
	str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

int main()
{
	std::string csv_path = "/home/sit/decode_jousi.csv";
	std::ifstream ifs(csv_path, std::ios_base::out);
	std::string read_line;
	std::getline(ifs, read_line);
	std::vector<std::string> cels = split(read_line);
	ifs.close();

	std::vector<uint8_t> raw_data;
	for(int i=0; i<cels.size(); i++)
	{
		//std::cout << std::stoi(cels[i], nullptr, 16) << std::endl;
		raw_data.push_back((uint8_t)std::stoi(cels[i], nullptr, 16));
	}

	
	i2v::READER reader(raw_data.data());
	reader.print();
/*		uint8_t use_load_id_ = 2;
		uint8_t use_car_signal_id_ = 3;
		uint8_t use_arrow_ = 0;
		if(reader.getServeceInfoSize() >= 1)
		{
			i2v::ServiceLoadSignalInfo service_info = reader.getServiceInfo(use_load_id_-1);
			std::cout << "service_info:" << +service_info.getID() << std::endl;
			std::cout << "getCarSignalInfoSize:" << service_info.getCarSignalInfoSize() << std::endl;
			if(service_info.getCarSignalInfoSize() >= use_car_signal_id_ && use_car_signal_id_!=0)
			{
				i2v::CarSignalInfo1 car_signal_info1 = service_info.getCarSignalInfo(use_car_signal_id_-1);
				std::cout << "car_signal_info1:" << +car_signal_info1.getID() << std::endl;
				if(car_signal_info1.getInfo2Size() > 0)
				{
					i2v::CarSignalInfo2 car_signal_info2 = car_signal_info1.getInfo2(0);
					std::cout << "car_signal_info2:" << +car_signal_info2.getCircleColorDisplay() << std::endl;
					int light;
					uint8_t circle = car_signal_info2.getCircleColorDisplay();
					uint8_t arrow = car_signal_info2.getArrowColorDisplay();
					if(circle == 0x1 || circle == 0x2)
					{
						light = TRAFFIC_LIGHT_GREEN;
					}
					else if(arrow == use_arrow_ && use_arrow_ != 0)
					{
						light = TRAFFIC_LIGHT_GREEN;
					}
					else
					{
						light = TRAFFIC_LIGHT_RED;
					}
					std::cout << "light:" << light << std::endl;
					std::cout << "min:" << +car_signal_info2.getMinRemainingSec10m() << std::endl;
					std::cout << "max:" << +car_signal_info2.getMaxRemainingSec10m() << std::endl;
				}
			}
		}*/

	reader.print();
}
