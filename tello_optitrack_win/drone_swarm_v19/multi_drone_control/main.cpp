#include"configuration.h"
#include"drone_swarm.h"
#include<string>
int main()
{
	drone_swarm::drone_swarm swarm;
	swarm.init_params();
	drone_swarm::run_receive_optitrack_data(swarm);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));


	swarm.run_receive_state_from_tellos();
	swarm.run_process_high_command_s();
	swarm.run_external_control_s();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	//for (unsigned int i = 0; i < swarm.params.number; i++)
	//{
	//	swarm.drones[i].arming();
	//}
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	/*for (unsigned int i = 0; i < swarm.params.number; i++)
	{
		swarm.drones[i].takeoff();
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));

*/
	//// circle motion

	//std::chrono::system_clock::time_point ts = std::chrono::system_clock::now();
	//std::chrono::system_clock::time_point te = std::chrono::system_clock::now();
	//long long delta_T = std::chrono::duration_cast<std::chrono::milliseconds>(te - ts).count();
	//float r = 1.0f;
	//float T = 10.0f;
	//float x_d = 0.0f;
	//float y_d = 0.0f;
	//float z_d = 0.5f;
	//float yaw_d = 0.0f;
	//while (delta_T < 20000)
	//{
	//	for (unsigned int i = 0; i < swarm.params.number; i++)
	//	{
	//		x_d = r * cos(2.0f*3.1415926f*(((float)delta_T) / 1000.0f / T + (float)i / (float)swarm.params.number));
	//		y_d = r * sin(2.0f*3.1415926f*(((float)delta_T) / 1000.0f / T + (float)i / (float)swarm.params.number));
	//		swarm.drones[i].setpoint_position(x_d, y_d, z_d, yaw_d);
	//	}
	//	std::this_thread::sleep_for(std::chrono::milliseconds(30));
	//	te = std::chrono::system_clock::now();
	//	delta_T = std::chrono::duration_cast<std::chrono::milliseconds>(te - ts).count();
	//}
	////

	//for (unsigned int i = 0; i < swarm.params.number; i++)
	//{
	//	swarm.drones[i].land_without_ep();
	//}
	//std::this_thread::sleep_for(std::chrono::milliseconds(5000));

	// Stop
	printf("===================================\n");
	printf("Press any key to exit.\n");
	printf("===================================\n");
	getchar();
	printf("Exiting......");

	swarm.stop();
	drone_swarm::mocap_stop();
	return 1;
}