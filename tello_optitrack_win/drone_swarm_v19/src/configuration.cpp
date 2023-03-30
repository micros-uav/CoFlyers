#pragma once
#include "configuration.h"
#include<fstream>
#include <string>

namespace config
{
	bool get_params_controller_from_txt(params_controller & params, const char* file_name)
	{
		// Open txt
		std::ifstream myfile(file_name);
		std::string string_line;
		if (!myfile.is_open())
		{
			printf("Error: cannot open %s.\n",file_name);
			return 0;
		}
		// Read txt
		float * ptr = (float *)&params;
		unsigned int count = 0;
		while (!myfile.eof())
		{
			count++;
			getline(myfile, string_line);
			try
			{
				*ptr = std::stof(string_line);
				ptr++;
				
			}
			catch (const std::exception&)
			{
				printf("Error: no float data in line %d!\n",count);
				break;
			}
		}
		return true;
	}

	bool get_params_swarm_from_txt(params_swarm & params, const char * file_name)
	{
		// Open txt
		std::ifstream myfile(file_name);
		std::string string_line;
		if (!myfile.is_open())
		{
			printf("Error: cannot open %s.\n",file_name);
			return false;
		}
		// Read txt
		getline(myfile, string_line);
		params.flag_tune_controller = (bool)std::stoi(string_line);
		printf("Tune pid controller: %d.\n", params.flag_tune_controller);

		getline(myfile, string_line);
		params.number = std::stoi(string_line);
		printf("The number of drones is %d.\n", params.number);

		getline(myfile, string_line);
		params.height_takeoff = std::stof(string_line);
		printf("The default height of take-off is %f.\n", params.height_takeoff);

		getline(myfile, string_line);
		strcpy(params.ip_local_mocap, string_line.c_str());
		printf("The local ip to communicate with mocap is %s.\n", params.ip_local_mocap);

		getline(myfile, string_line);
		strcpy(params.ip_target_mocap, string_line.c_str());
		printf("The target ip to communicate with mocap is %s.\n", params.ip_target_mocap);

		getline(myfile, string_line);
		params.frame_num = (unsigned short int)std::stoi(string_line);
		printf("The frame number of mocap is %d.\n", params.frame_num);

		getline(myfile, string_line);
		params.span_filter_v = (unsigned int)std::stoi(string_line);
		printf("The span for sliding-window filter of velocity is %d.\n", params.span_filter_v);

		getline(myfile, string_line);
		params.span_filter_a = (unsigned int)std::stoi(string_line);
		printf("The span for sliding-window filter of acceleration is %d.\n", params.span_filter_a);

		getline(myfile, string_line);
		params.flag_save_data = (bool)std::stoi(string_line);
		printf("Save data: %d.\n", params.flag_save_data);

		getline(myfile, string_line);
		strcpy(params.ip_local_external, string_line.c_str());
		printf("The local ip to communicate with external commander is %s.\n", params.ip_local_external);

		getline(myfile, string_line);
		strcpy(params.ip_target_external, string_line.c_str());
		printf("The target ip to communicate with external commander is %s.\n", params.ip_target_external);

		getline(myfile, string_line);
		params.port_local_start = (unsigned short int)std::stoi(string_line);
		printf("The local start port to communicate with external commander is %d.\n", params.port_local_start);
		printf("Note that: local: port_local_start +1 + 2*i, target_state: port_local_start + 2*i, target_bar: port_local_start - 1000 + 2*i. i = 0-number.\n");

		getline(myfile, string_line);
		strcpy(params.ip_local_drone, string_line.c_str());
		printf("The local ip to communicate with drones is %s.\n", params.ip_local_drone);

		params.ip_target_drones = new char*[params.number];
		for (int i = 0; i < params.number; i++)
		{
			params.ip_target_drones[i] = new char[16];
			memset(params.ip_target_drones[i], 0, 16);
		}
		int countNum = 0;
		while (getline(myfile, string_line))
		{
			if (countNum + 1 > params.number)
			{
				printf("Warning: the declared number is smaller than the number of drone ip!\n");
				break;
			}
			//tellos_ip[countNum] = string_line;
			memcpy(params.ip_target_drones[countNum], string_line.c_str(), 16);
			printf("The ip of No.%d drone is %s\n", countNum, params.ip_target_drones[countNum]);
			countNum++;
		}
		myfile.close();
		if (countNum < params.number)
		{
			printf("Error: the declared number of ip is less than %d.\n",params.number);
			return false;
		}
		return true;
	}


}