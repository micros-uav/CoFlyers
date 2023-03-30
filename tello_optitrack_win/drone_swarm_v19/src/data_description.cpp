#include"data_description.h"
namespace msg
{

	int getMsgNum(unsigned char componentId, unsigned char  messageId)
	{
		int num = 0;
		switch (componentId)
		{
		case  COMMAND_TYPE:
			/* Control */
			switch (messageId)
			{
			case  msg::SET_MODE_TYPE:
				/* Set mode */
				num = SET_MODE_TYPE_NUM;
				break;
			case  msg::ARMING_TYPE:
				/* Arming */
				num = ARMING_TYPE_NUM;
				break;
			case  TAKEOFF_TYPE:
				/* Takeoff */
				num = TAKEOFF_TYPE_NUM;
				break;
			case  HOVER_TYPE:
				/* Hover */
				num = HOVER_TYPE_NUM;
				break;
			case  LAND_TYPE:
				/* Land */
				num = LAND_TYPE_NUM;
				break;
			case  POSITION_CONTROL_TYPE:
				/* Position control */
				num = POSITION_CONTROL_TYPE_NUM;
				break;
			case  VELOCITY_CONTROL_TYPE:
				/* Velocity control */
				num = VELOCITY_CONTROL_TYPE_NUM;
				break;
			case  VELOCITY_HORIZONTAL_CONTROL_TYPE:
				/* Velocity horizontal control */
				num = VELOCITY_HORIZONTAL_CONTROL_TYPE_NUM;
				break;
			case  ACCELERATION_CONTROL_TYPE:
				/* Acceleration control */
				num = ACCELERATION_CONTROL_TYPE_NUM;
				break;
			case  ATTITUDE_CONTROL_TYPE:
				/* Attitude control */
				num = ATTITUDE_CONTROL_TYPE_NUM;
				break;
			case FULL_STATE_CONTROL_TYPE:
				num = FULL_STATE_CONTROL_TYPE_NUM;
				break;
			default:
				num = ERROR_MSG_NUM;
				break;
			}
			break;
		case  STATE_TYPE:
			/*  */
			switch (messageId)
			{
			case  KINEMATIC_STATE_TYPE:
				/* */
				num = KINEMATIC_STATE_TYPE_NUM;
				break;
			case  BATTERY_STATE_TYPE:
				/* */
				num = BATTERY_STATE_TYPE_NUM;
				break;
			case SIMPLE_STATE_TYPE:
				num = SIMPLE_STATE_TYPE_NUM;
				break;
			case STATE_12_TYPE:
				num = STATE_12_TYPE_NUM;
				break;
			default:
				num = ERROR_MSG_NUM;
				break;
			}
			break;
		default:
			num = ERROR_MSG_NUM;
			break;
		}
		return num;
	}

	unsigned short int get_check_num(const package& myPackage)
	{
		unsigned short int checkNum = 111;
		// myPackage
		return checkNum;
	}
}