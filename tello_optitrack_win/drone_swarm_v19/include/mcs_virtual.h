#pragma once
#include"data_description.h"

namespace mcs
{
	class Motion_capture_system
	{
	public:
		virtual int activate(const char* ip_server, const char* ip_client) = 0;
		virtual void set_process_rigid_data_handler(void(*process_rigid_data_handler1)(const unsigned int id, const data_description::state_rigidbody* state)) = 0;
		virtual void set_timestamp_handler(void(*get_timestamp_handler1)(const double)) = 0;
		virtual void stop() = 0;
	private:

	};

}