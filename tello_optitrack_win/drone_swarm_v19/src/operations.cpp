#include "..\include\operations.h"
#include<math.h>
namespace operations
{
	const float pi = 3.1415926f;

	void quaternion2euler(const data_description::quaternion & q, data_description::euler & e)
	{
		e.pitch = (float)atan2(-2.0f * q.qx*q.qz + 2.0f * q.qw*q.qy, -2.0f * q.qx*q.qx - 2.0f * q.qy*q.qy + 1.0f) * 180.0f/pi;
		e.roll = (float)asin(2.0f * q.qw*q.qx + 2.0f * q.qy*q.qz)  * 180.0f / pi;
		e.yaw = (float)atan2(-2.0f * q.qx*q.qy + 2.0f * q.qw*q.qz, -2.0f * q.qx*q.qx - 2.0f * q.qz*q.qz + 1.0f) * 180.0f / pi;
	}
}