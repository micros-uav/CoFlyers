#pragma once

#include "lib_track_api.h"
#include <stdio.h>
#pragma comment(lib,"WS2_32.lib")
#if defined( __WIN32__ ) || defined( WIN32 ) || defined( _WIN32 )
#include <Windows.h>
#include <conio.h>
#include <crtdbg.h>
//memery leak check
#ifdef _DEBUG
#define new   new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif


inline void EnableMemLeakCheck()
{
	_CrtSetDbgFlag(_CrtSetDbgFlag(_CRTDBG_REPORT_FLAG) | _CRTDBG_LEAK_CHECK_DF);
}

LARGE_INTEGER eticks, frequency;
int64_t get_time_ms() {
	QueryPerformanceCounter(&eticks);
	return (eticks.QuadPart * 1000 / frequency.QuadPart);
}

char* GetErrMsg(DWORD errCode)
{
	LPVOID lpMsgBuf;
	if (errCode == 0)
		errCode = GetLastError();
	DWORD dwR = FormatMessageA(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
		NULL,
		errCode,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPSTR)&lpMsgBuf,
		0, NULL);

	if (dwR == 0)
		return (char*)"NULL";
	return (char*)lpMsgBuf;
}

#else
#include <dlfcn.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>

#define HMODULE		void*
#define _snprintf_s	snprintf
#define strcat_s	strcat
inline void EnableMemLeakCheck()
{
}

int64_t get_time_ms() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
#endif

////
#include"mcs_virtual.h"
#include<thread>
////

namespace zvr
{
	///library handle
	HMODULE	handle = NULL;
	///for show info
	uint32_t	g_uTotalCurr = 0;
	uint32_t	g_uTotalLast = 0;
	int64_t		g_tsLast = 0;

	///call back function
	void /*__cdecl*/ DataCallBack(void* pinfo, void* pdata);



	void unload_model()
	{
		if (handle)
		{
#if defined( __WIN32__ ) || defined( WIN32 ) || defined( _WIN32 )
			FreeLibrary(handle);
#else
			dlclose(handle);
#endif
			handle = NULL;
		}
	}

	TrackApi* load_model(char* path)
	{
		char szFilePath[256];
#if defined( __WIN32__ ) || defined( WIN32 ) || defined( _WIN32 )
		if (path == NULL) {
			GetModuleFileNameA(NULL, szFilePath, 256);
			char* pszRoot = strrchr(szFilePath, '\\');
			pszRoot += 1;
			strcpy_s(pszRoot, 13, "libGokuC.dll");
		}
		else
		{
			sprintf_s(szFilePath, "%slibGokuC.dll", path);
		}

		CTOR_FUNPTR Constructor;
		handle = LoadLibraryA(szFilePath);
		if (handle)
		{
			if (!(Constructor = (CTOR_FUNPTR)GetProcAddress(handle, "GetConstructor")))
			{
				unload_model();
				printf(GetErrMsg(GetLastError()));
				return NULL;
			}
		}
		else {
			printf("%s\n", GetErrMsg(GetLastError()));
			return NULL;
		}

		return (TrackApi*)Constructor();
#else
		int cnt = readlink("/proc/self/exe", szFilePath, 256);
		if (cnt < 0 || cnt >= 256)
		{
			return NULL;
		}

		char* pszRoot = strrchr(szFilePath, '/');
		pszRoot += 1;
		strncpy(pszRoot, "libGokuC.so", 12);

		handle = dlopen(szFilePath, RTLD_LAZY);
		//handle = dlopen("libGokuC.so", RTLD_LAZY);
		if (handle == NULL)
		{
			printf("dlopen libGokuC.so failed\n");
			return NULL;
		}

		CTOR_FUNPTR Constructor = (CTOR_FUNPTR)dlsym(handle, "GetConstructor");
		if (Constructor)
		{
			printf("dlsym GetConstructor ok.\n");
		}
		else
		{
			printf("dlsym GetConstructor failed.\n");
			unload_model();
		}

		return (TrackApi*)Constructor();
#endif
	}

	void _init_()
	{
#if defined( __WIN32__ ) || defined( WIN32 ) || defined( _WIN32 )
		EnableMemLeakCheck();
		QueryPerformanceFrequency(&frequency);
#else
#endif
	}

	TrackApi* pTrack = load_model(NULL);

	int activate_zvr(const char* ip_server, const char* ip_client)
	{
		_init_();
		bool bExit = false;
		if (pTrack == NULL)
			return 0;

		//get info
		printf("%s", pTrack->getdesc());
		//set callback function
		pTrack->SetDataCallback((PFCALLBACK)DataCallBack);

		//start as client
		//int nR = pTrack->start((char*)"239.8.192.168", (char*)"192.168.1.29", 15515, 15516);//server ip,local ip,command port,data port
		int nR = pTrack->start((char*)ip_server, (char*)ip_client, 15515, 15516);//server ip,local ip,command port,data port
		//start as simulate server
		//int nR = pTrack->start((char *)"192.168.1.9", (char *)"239.8.192.168", 15515, 15516,true);//server ip,local ip,command port,data port

		if (nR != 0)
		{
			printf("pTrack start error\n");
			bExit = true;
			return -1;
		}

		/*int c;
		while ((!bExit) && (c = getchar()))
		{
			switch (c)
			{
			case 'q':
				bExit = true;
				break;
			default:
				break;
			}
		}*/


		return 0;
	}

	int stop_zvr()
	{
		if (pTrack) {
			pTrack->stop();
			delete pTrack;
			pTrack = NULL;
		}

		unload_model();

		return 0;
	}

	class mcs_zvr: public mcs::Motion_capture_system
	{
	public:
		mcs_zvr()
		{

		}
		~mcs_zvr()
		{

		}
		int activate(const char* ip_server, const char* ip_client)
		{
			//this->thread_receive_data_from_mcs = std::thread(activate_zvr,ip_server,ip_client);
			return activate_zvr(ip_server, ip_client);
		}
		void set_process_rigid_data_handler(void(*process_rigid_data_handler_in)(const unsigned int id, const data_description::state_rigidbody* state))
		{
			this->process_rigid_data_handler = process_rigid_data_handler_in;
		}
		void set_timestamp_handler(void(*get_timestamp_handler_in)(const double))
		{
			this->get_timestamp_handler = get_timestamp_handler_in;
		}
		void stop()
		{
			stop_zvr();
		}
		friend void DataCallBack(void* pinfo, void* pdata);
	private:
		void(*process_rigid_data_handler)(const unsigned int id, const data_description::state_rigidbody* state) = nullptr;
		void(*get_timestamp_handler)(const double timestamp) = nullptr;
	}mcs_zvr_handle;

	void /*__cdecl*/ DataCallBack(void* pinfo, void* pdata)
	{
		data_description::state_rigidbody temp;
		info_t* pi = (info_t*)pinfo;
		if (pi->count != 0)
		{
			char szmsg[1024];
			_snprintf_s(szmsg, 1024, "[%ld][%d][%f][%d]:", get_time_ms(), pi->iFrame, pi->fTimestamp, pi->count);
			orientation_t* pd = (orientation_t*)pdata;
			char szt[32];
			for (int i = 0; i < pi->count; i++)
			{
				_snprintf_s(szt, 32, "[%d]", pd->id);
				strcat_s(szmsg, szt);
				temp.pos.x = pd->pos.x;
				temp.pos.y = pd->pos.y;
				temp.pos.z = pd->pos.z;
				temp.q.qw = pd->quat.w;
				temp.q.qx = pd->quat.x;
				temp.q.qy = pd->quat.y;
				temp.q.qz = pd->quat.z;
				mcs_zvr_handle.process_rigid_data_handler(pd->id-1, &temp);
				pd += 1;
			}
			//printf("%s\n", szmsg);
		}

		if (g_uTotalCurr == 0) {
			g_tsLast = get_time_ms();
		}

		g_uTotalCurr++;
		uint32_t udiff = g_uTotalCurr - g_uTotalLast;
		if (udiff > 100) {
			long long   ts = get_time_ms();
			long long ts_diff = ts - g_tsLast;
			if (ts_diff != 0) {
				//printf("--fps is %f--\n", 1000.f * udiff / ts_diff);
			}
			g_uTotalLast = g_uTotalCurr;
			g_tsLast = ts;
			
		}
		mcs_zvr_handle.get_timestamp_handler((double)get_time_ms());
	}

}