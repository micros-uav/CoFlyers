#ifndef LIB_TRACK_API
#define LIB_TRACK_API

#define	MODULE	extern "C"

#include <stdint.h>

MODULE typedef void* (*CTOR_FUNPTR)(void);
typedef void (*PFCALLBACK)(void *pinfo, void *pdata);

#pragma  pack(push) 
#pragma  pack(1)  
//euler angle for rotation in 3d
struct euler_t{
	float pitch;
	float yaw;
	float roll;
};

//quaternion for rotation in 3d
struct quaternion_t{
	float x;
	float y;
	float z;
	float w;
};

//position in 3d
struct position_t{
	float x;
	float y;
	float z;
};

//rigidbody struct
struct orientation_t{
	int id;
	union{
		struct { quaternion_t quat; position_t pos; };
		struct { float wx, wy, wz, ww, x, y, z; };
	};
	bool bTrigger;
};

struct info_t{
	uint8_t		uType;			//PACKET_TYPE
	uint16_t 	size;			//total size of packet
	uint32_t 	iFrame;			//frame id
	double 		fTimestamp;		//ts calculate from frame id fTimestamp=frame_id/FPS
	uint16_t 	count;			//count of rigidbody
	uint8_t 	uFlag;			//not use
};

#pragma pack(pop)

class  TrackApi
{
public:
	virtual ~TrackApi() {}

	/*!
	*	use param start
	*	@param[in] char *szSrvAddr	multicast ip,same as ActiveCenter
	*	@param[in] char *szLocAddr	local ip which connected with ActiveCenter
	*	@param[in] int cmdPort		same as ActiveCenter
	*	@param[in] int dataPort		same as ActiveCenter
	*	@param[in] bool bAsSrv		true simulate AC send for test
	*	@return		int				0 success
	*/
	virtual int start(char *szSrvAddr, char *szLocAddr, int cmdPort, int dataPort,bool bAsSrv=false) = 0;

	/*!
	*	stop
	*/
	virtual void stop() = 0;

	/*!
	*	set callback,process the recved data
	*/
	virtual int SetDataCallback(PFCALLBACK pcallback) = 0;

	/*!
	*	get sdk info
	*/
	virtual char *getdesc() = 0;
};

#endif