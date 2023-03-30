#pragma once

#ifndef _DATADESCRIPTION
#define _DATADESCRIPTION
namespace data_description
{
	struct position
	{
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
	};

	struct quaternion
	{
		float qx = 0.0f;
		float qy = 0.0f;
		float qz = 0.0f;
		float qw = 1.0f;
	};

	struct euler
	{
		float pitch = 0.0f;
		float roll = 0.0f;
		float yaw = 0.0f;
	};

	struct state_simple
	{
		position pos;
		position vel;
		euler angle;
	};

	struct state_rigidbody
	{
		position pos;
		quaternion q;
	};

	struct attitude_command
	{
		float pitch = 0.0f;
		float roll = 0.0f;
		float yaw_rate = 0.0f;
		float thrust = 0.0f;
	};
}


namespace msg
{
	constexpr int LEN_MAX_MSG = 16;
	constexpr int LEN_MAX_BUFFER = LEN_MAX_MSG * 4 + 16;
	
	struct package
	{
		unsigned char start = 0xFA;//#0
		unsigned char lenPayload = 0;  //#1
		unsigned char numSequence = 0; //#2
		unsigned char systemId = 0;    //#3
		unsigned char componentId = 0; //#4
		unsigned char messageId = 0;   //#5
		float msgData[LEN_MAX_MSG]{}; //#6-n+5
		double timeStamp = 0.0;    //#n+6 - n+13
		unsigned short int checkNum = 111;    //#n+14 - n+15
	};

	///////////////////////Package type//////////////////////
#ifndef HAVE_ENUM_COMPONENT_TYPE
#define HAVE_ENUM_COMPONENT_TYPE
	typedef enum COMPONENT_TYPE
	{
		COMMAND_TYPE = 0,
		STATE_TYPE = 1
	} COMPONENT_TYPE;
#endif

#ifndef HAVE_ENUM_MESSAGE_TYPE_COMMAND_TYPE
#define HAVE_ENUM_MESSAGE_TYPE_COMMAND_TYPE
	typedef enum MESSAGE_TYPE_COMMAND_TYPE
	{
		SET_MODE_TYPE = 0,
		ARMING_TYPE = 1,
		TAKEOFF_TYPE = 2,
		HOVER_TYPE = 3,
		LAND_TYPE = 4,
		POSITION_CONTROL_TYPE = 5,
		VELOCITY_CONTROL_TYPE = 6,
		VELOCITY_HORIZONTAL_CONTROL_TYPE = 7,
		ACCELERATION_CONTROL_TYPE = 8,
		ATTITUDE_CONTROL_TYPE = 9,
		FULL_STATE_CONTROL_TYPE = 10
	} MESSAGE_TYPE_COMMAND_TYPE;
#endif

#ifndef HAVE_ENUM_MESSAGE_TYPE_STATE_TYPE
#define HAVE_ENUM_MESSAGE_TYPE_STATE_TYPE
	typedef enum MESSAGE_TYPE_STATE_TYPE
	{
		KINEMATIC_STATE_TYPE = 0,
		BATTERY_STATE_TYPE = 1,
		SIMPLE_STATE_TYPE = 2,
		STATE_12_TYPE = 3
	} MESSAGE_TYPE_STATE_TYPE;
#endif

#ifndef HAVE_ENUM_MESSAGE_TYPE_COMMAND_TYPE_NUM
#define HAVE_ENUM_MESSAGE_TYPE_COMMAND_TYPE_NUM
	typedef enum MESSAGE_TYPE_COMMAND_TYPE_NUM
	{
		SET_MODE_TYPE_NUM = 2,
		ARMING_TYPE_NUM = 1,
		TAKEOFF_TYPE_NUM = 1,
		HOVER_TYPE_NUM = 1,
		LAND_TYPE_NUM = 1,
		POSITION_CONTROL_TYPE_NUM = 4,
		VELOCITY_CONTROL_TYPE_NUM = 4,
		VELOCITY_HORIZONTAL_CONTROL_TYPE_NUM = 4,
		ACCELERATION_CONTROL_TYPE_NUM = 4,
		ATTITUDE_CONTROL_TYPE_NUM = 4,
		FULL_STATE_CONTROL_TYPE_NUM = 16
	} MESSAGE_TYPE_COMMAND_TYPE_NUM;
#endif

#ifndef HAVE_ENUM_MESSAGE_TYPE_STATE_TYPE_NUM
#define HAVE_ENUM_MESSAGE_TYPE_STATE_TYPE_NUM
	typedef enum MESSAGE_TYPE_STATE_TYPE_NUM
	{
		KINEMATIC_STATE_TYPE_NUM = 16,
		BATTERY_STATE_TYPE_NUM = 1,
		SIMPLE_STATE_TYPE_NUM = 9,
		STATE_12_TYPE_NUM = 12
	} MESSAGE_TYPE_STATE_TYPE_NUM;
#endif


#define ERROR_MSG_NUM -1

	// Get the number of message according to the component id and the message id.
	int getMsgNum(unsigned char componentId, unsigned char  messageId);
	// Get the check number according to the package.
	unsigned short int get_check_num(const package& myPackage);

	//////////////////////////////////////////////////////////
}

#endif
