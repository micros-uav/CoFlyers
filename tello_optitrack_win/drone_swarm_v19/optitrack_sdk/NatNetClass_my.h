#pragma once
/*
Copyright © 2012 NaturalPoint Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */


/*

SampleClient.cpp

This program connects to a NatNet server, receives a data stream, and writes that data stream
to an ascii file.  The purpose is to illustrate using the NatNetClient class.

Usage [optional]:

	SampleClient [ServerIP] [LocalIP] [OutputFilename]

	[ServerIP]			IP address of the server (e.g. 192.168.0.107) ( defaults to local machine)
	[OutputFilename]	Name of points file (pts) to write out.  defaults to Client-output.pts

*/

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#   include <conio.h>
#else
#   include <unistd.h>
#   include <termios.h>
#endif

#include <vector>
//
//#include <NatNetTypes.h>
//#include <NatNetCAPI.h>
//#include <NatNetClient.h>
#include "NatNetTypes.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"

//////////////
#include "data_description.h"
////////////


namespace natnet
{
	///////////////////User///////////////////////
	void set_process_rigid_data_handler(void(*process_rigid_data_handler1)(const unsigned int id, const data_description::state_rigidbody* state));
	void set_timestamp_handler(void(*get_timestamp_handler1)(const double));

	void(*process_rigid_data_handler)(const unsigned int id, const data_description::state_rigidbody* state) = nullptr;
	void(*get_timestamp_handler)(const double timestamp) = nullptr;
	
	data_description::state_rigidbody *state_ptr = nullptr;

	void set_process_rigid_data_handler(void(*process_rigid_data_handler1)(const unsigned int id, const data_description::state_rigidbody* state))
	{
		process_rigid_data_handler = process_rigid_data_handler1;
}
	void set_timestamp_handler(void(*get_timestamp_handler1)(const double))
	{
		get_timestamp_handler = get_timestamp_handler1;
	}

	/////////////////////////////////////////////
#ifndef _WIN32
	char getch();
#endif
	void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
	void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
	void _WriteFooter(FILE* fp);
	void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext);
	void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
	void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages
	void resetClient();
	int ConnectClient();

	static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

	NatNetClient* g_pClient = NULL;
	FILE* g_outputFile;

	std::vector< sNatNetDiscoveredServer > g_discoveredServers;
	sNatNetClientConnectParams g_connectParams;
	char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
	int g_analogSamplesPerMocapFrame = 0;
	sServerDescription g_serverDescription;

	int activate(const char * ip_server, const char * ip_client);
	int stop();

	int stop()
	{
		// Done - clean up.
		if (g_pClient)
		{
			g_pClient->Disconnect();
			delete g_pClient;
			g_pClient = NULL;
		}

		if (g_outputFile)
		{
			_WriteFooter(g_outputFile);
			fclose(g_outputFile);
			g_outputFile = NULL;
		}
		return ErrorCode_OK;
	}

	int activate(const char * ip_server, const char * ip_client)
	{
		// print version info
		unsigned char ver[4];
		NatNet_GetVersion(ver);
		printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

		// Install logging callback
		NatNet_SetLogCallback(MessageHandler);

		// create NatNet client
		g_pClient = new NatNetClient();

		// set the frame callback handler
		g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient);	// this function will receive data from the server


		g_connectParams.connectionType = kDefaultConnectionType;
		g_connectParams.serverAddress = ip_server;
		g_connectParams.localAddress = ip_client;

		int iResult;
		// Connect to Motive
		iResult = ConnectClient();
		if (iResult != ErrorCode_OK)
		{
			printf("Error initializing client. See log for details. Exiting.\n");
			return 1;
		}
		else
		{
			printf("Client initialized and ready.\n");
		}

		return ErrorCode_OK;
	}


	void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext)
	{
		char serverHotkey = '.';
		if (g_discoveredServers.size() < 9)
		{
			serverHotkey = static_cast<char>('1' + g_discoveredServers.size());
		}

		printf("[%c] %s %d.%d at %s ",
			serverHotkey,
			pDiscoveredServer->serverDescription.szHostApp,
			pDiscoveredServer->serverDescription.HostAppVersion[0],
			pDiscoveredServer->serverDescription.HostAppVersion[1],
			pDiscoveredServer->serverAddress);

		if (pDiscoveredServer->serverDescription.bConnectionInfoValid)
		{
			printf("(%s)\n", pDiscoveredServer->serverDescription.ConnectionMulticast ? "multicast" : "unicast");
		}
		else
		{
			printf("(WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)\n");
		}

		g_discoveredServers.push_back(*pDiscoveredServer);
	}

	// Establish a NatNet Client connection
	int ConnectClient()
	{
		// Release previous server
		g_pClient->Disconnect();

		// Init Client and connect to NatNet server
		int retCode = g_pClient->Connect(g_connectParams);
		if (retCode != ErrorCode_OK)
		{
			printf("Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
			return ErrorCode_Internal;
		}
		else
		{
			// connection succeeded

			void* pResult;
			int nBytes = 0;
			ErrorCode ret = ErrorCode_OK;

			// print server info
			memset(&g_serverDescription, 0, sizeof(g_serverDescription));
			ret = g_pClient->GetServerDescription(&g_serverDescription);
			if (ret != ErrorCode_OK || !g_serverDescription.HostPresent)
			{
				printf("Unable to connect to server. Host not present. Exiting.\n");
				return 1;
			}
			printf("\n[SampleClient] Server application info:\n");
			printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
				g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
			printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
				g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
			printf("Client IP:%s\n", g_connectParams.localAddress);
			printf("Server IP:%s\n", g_connectParams.serverAddress);
			printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

			// get mocap frame rate
			ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
			if (ret == ErrorCode_OK)
			{
				float fRate = *((float*)pResult);
				printf("Mocap Framerate : %3.2f\n", fRate);
			}
			else
				printf("Error getting frame rate.\n");

			// get # of analog samples per mocap frame of data
			ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
			if (ret == ErrorCode_OK)
			{
				g_analogSamplesPerMocapFrame = *((int*)pResult);
				printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
			}
			else
				printf("Error getting Analog frame rate.\n");
		}

		return ErrorCode_OK;
	}



	double fTimestamp_pre = 0.0;
	double dTimestamp = 0.0;
	// DataHandler receives data from the server
	// This function is called by NatNet when a frame of mocap data is available
	void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
	{

		if (get_timestamp_handler != nullptr)
		{
			get_timestamp_handler(data->fTimestamp);
		}
		int i = 0;

		//dTimestamp = data->fTimestamp - fTimestamp_pre;
		//printf("FrameID : %d\n", data->iFrame);
		//printf("Timestamp : %3.2lf\n", data->fTimestamp);
		//printf("dTimestamp : %1.8lf\n", dTimestamp);
		//fTimestamp_pre = data->fTimestamp;
		//// Rigid Bodies
		//printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
		
		for (i = 0; i < data->nRigidBodies; i++)
		{
			// params
			// 0x01 : bool, rigid body was successfully tracked in this frame
			bool bTrackingValid = data->RigidBodies[i].params & 0x01;

			/*printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
			printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
			printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
				data->RigidBodies[i].x,
				data->RigidBodies[i].y,
				data->RigidBodies[i].z,
				data->RigidBodies[i].qx,
				data->RigidBodies[i].qy,
				data->RigidBodies[i].qz,
				data->RigidBodies[i].qw);*/
			if (process_rigid_data_handler != nullptr && bTrackingValid)
			{
				state_ptr = (data_description::state_rigidbody*)&(data->RigidBodies[i].x);
				//printf("ID:%d,x:%f\n",data->RigidBodies[i].ID,state_ptr->x);
				process_rigid_data_handler(data->RigidBodies[i].ID,state_ptr);
			}
		}
		
	}


	//void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
	//{
	//	NatNetClient* pClient = (NatNetClient*)pUserData;

	//	// Software latency here is defined as the span of time between:
	//	//   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
	//	// and
	//	//   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
	//	//
	//	// This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
	//	// because it additionally includes the time spent preparing to stream the data via NatNet.
	//	const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
	//	const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

	//	// Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
	//	// The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
	//	const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

	//	if (g_outputFile)
	//	{
	//		_WriteFrame(g_outputFile, data);
	//	}

	//	int i = 0;

	//	printf("FrameID : %d\n", data->iFrame);
	//	printf("Timestamp : %3.2lf\n", data->fTimestamp);
	//	printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);

	//	// Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
	//	// If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
	//	const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

	//	if (bSystemLatencyAvailable)
	//	{
	//		// System latency here is defined as the span of time between:
	//		//   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
	//		// and
	//		//   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
	//		const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
	//		const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

	//		// Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
	//		// This is the all-inclusive measurement (photons to client processing).
	//		const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp) * 1000.0;

	//		// You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
	//		//const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

	//		printf("System latency : %.2lf milliseconds\n", systemLatencyMillisec);
	//		printf("Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec);
	//	}
	//	else
	//	{
	//		printf("Transit latency : %.2lf milliseconds\n", transitLatencyMillisec);
	//	}

	//	// FrameOfMocapData params
	//	bool bIsRecording = ((data->params & 0x01) != 0);
	//	bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
	//	if (bIsRecording)
	//		printf("RECORDING\n");
	//	if (bTrackedModelsChanged)
	//		printf("Models Changed.\n");


	//	// timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	//	int hour, minute, second, frame, subframe;
	//	NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
	//	// decode to friendly string
	//	char szTimecode[128] = "";
	//	NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
	//	printf("Timecode : %s\n", szTimecode);

	//	// Rigid Bodies
	//	printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
	//	for (i = 0; i < data->nRigidBodies; i++)
	//	{
	//		// params
	//		// 0x01 : bool, rigid body was successfully tracked in this frame
	//		bool bTrackingValid = data->RigidBodies[i].params & 0x01;

	//		printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
	//		printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
	//		printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
	//			data->RigidBodies[i].x,
	//			data->RigidBodies[i].y,
	//			data->RigidBodies[i].z,
	//			data->RigidBodies[i].qx,
	//			data->RigidBodies[i].qy,
	//			data->RigidBodies[i].qz,
	//			data->RigidBodies[i].qw);
	//	}

	//	// Skeletons
	//	printf("Skeletons [Count=%d]\n", data->nSkeletons);
	//	for (i = 0; i < data->nSkeletons; i++)
	//	{
	//		sSkeletonData skData = data->Skeletons[i];
	//		printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
	//		for (int j = 0; j < skData.nRigidBodies; j++)
	//		{
	//			sRigidBodyData rbData = skData.RigidBodyData[j];
	//			printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
	//				rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw);
	//		}
	//	}

	//	// labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
	//	bool bOccluded;     // marker was not visible (occluded) in this frame
	//	bool bPCSolved;     // reported position provided by point cloud solve
	//	bool bModelSolved;  // reported position provided by model solve
	//	bool bHasModel;     // marker has an associated asset in the data stream
	//	bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
	//	bool bActiveMarker; // marker is an actively labeled LED marker

	//	printf("Markers [Count=%d]\n", data->nLabeledMarkers);
	//	for (i = 0; i < data->nLabeledMarkers; i++)
	//	{
	//		bOccluded = ((data->LabeledMarkers[i].params & 0x01) != 0);
	//		bPCSolved = ((data->LabeledMarkers[i].params & 0x02) != 0);
	//		bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
	//		bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
	//		bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
	//		bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

	//		sMarker marker = data->LabeledMarkers[i];

	//		// Marker ID Scheme:
	//		// Active Markers:
	//		//   ID = ActiveID, correlates to RB ActiveLabels list
	//		// Passive Markers: 
	//		//   If Asset with Legacy Labels
	//		//      AssetID 	(Hi Word)
	//		//      MemberID	(Lo Word)
	//		//   Else
	//		//      PointCloud ID
	//		int modelID, markerID;
	//		NatNet_DecodeID(marker.ID, &modelID, &markerID);

	//		char szMarkerType[512];
	//		if (bActiveMarker)
	//			strcpy(szMarkerType, "Active");
	//		else if (bUnlabeled)
	//			strcpy(szMarkerType, "Unlabeled");
	//		else
	//			strcpy(szMarkerType, "Labeled");

	//		printf("%s Marker [ModelID=%d, MarkerID=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
	//			szMarkerType, modelID, markerID, marker.size, marker.x, marker.y, marker.z);
	//	}

	//	// force plates
	//	printf("Force Plate [Count=%d]\n", data->nForcePlates);
	//	for (int iPlate = 0; iPlate < data->nForcePlates; iPlate++)
	//	{
	//		printf("Force Plate %d\n", data->ForcePlates[iPlate].ID);
	//		for (int iChannel = 0; iChannel < data->ForcePlates[iPlate].nChannels; iChannel++)
	//		{
	//			printf("\tChannel %d:\t", iChannel);
	//			if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames == 0)
	//			{
	//				printf("\tEmpty Frame\n");
	//			}
	//			else if (data->ForcePlates[iPlate].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
	//			{
	//				printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->ForcePlates[iPlate].ChannelData[iChannel].nFrames);
	//			}
	//			for (int iSample = 0; iSample < data->ForcePlates[iPlate].ChannelData[iChannel].nFrames; iSample++)
	//				printf("%3.2f\t", data->ForcePlates[iPlate].ChannelData[iChannel].Values[iSample]);
	//			printf("\n");
	//		}
	//	}

	//	// devices
	//	printf("Device [Count=%d]\n", data->nDevices);
	//	for (int iDevice = 0; iDevice < data->nDevices; iDevice++)
	//	{
	//		printf("Device %d\n", data->Devices[iDevice].ID);
	//		for (int iChannel = 0; iChannel < data->Devices[iDevice].nChannels; iChannel++)
	//		{
	//			printf("\tChannel %d:\t", iChannel);
	//			if (data->Devices[iDevice].ChannelData[iChannel].nFrames == 0)
	//			{
	//				printf("\tEmpty Frame\n");
	//			}
	//			else if (data->Devices[iDevice].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
	//			{
	//				printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->Devices[iDevice].ChannelData[iChannel].nFrames);
	//			}
	//			for (int iSample = 0; iSample < data->Devices[iDevice].ChannelData[iChannel].nFrames; iSample++)
	//				printf("%3.2f\t", data->Devices[iDevice].ChannelData[iChannel].Values[iSample]);
	//			printf("\n");
	//		}
	//	}
	//}


	// MessageHandler receives NatNet error/debug messages
	void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg)
	{
		// Optional: Filter out debug messages
		if (msgType < Verbosity_Info)
		{
			return;
		}

		printf("\n[NatNetLib]");

		switch (msgType)
		{
		case Verbosity_Debug:
			printf(" [DEBUG]");
			break;
		case Verbosity_Info:
			printf("  [INFO]");
			break;
		case Verbosity_Warning:
			printf("  [WARN]");
			break;
		case Verbosity_Error:
			printf(" [ERROR]");
			break;
		default:
			printf(" [?????]");
			break;
		}

		printf(": %s\n", msg);
	}


	/* File writing routines */
	void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs)
	{
		int i = 0;

		if (pBodyDefs->arrDataDescriptions[0].type != Descriptor_MarkerSet)
			return;

		sMarkerSetDescription* pMS = pBodyDefs->arrDataDescriptions[0].Data.MarkerSetDescription;

		fprintf(fp, "<MarkerSet>\n\n");
		fprintf(fp, "<Name>\n%s\n</Name>\n\n", pMS->szName);

		fprintf(fp, "<Markers>\n");
		for (i = 0; i < pMS->nMarkers; i++)
		{
			fprintf(fp, "%s\n", pMS->szMarkerNames[i]);
		}
		fprintf(fp, "</Markers>\n\n");

		fprintf(fp, "<Data>\n");
		fprintf(fp, "Frame#\t");
		for (i = 0; i < pMS->nMarkers; i++)
		{
			fprintf(fp, "M%dX\tM%dY\tM%dZ\t", i, i, i);
		}
		fprintf(fp, "\n");

	}


	void _WriteFrame(FILE* fp, sFrameOfMocapData* data)
	{
		fprintf(fp, "%d", data->iFrame);
		for (int i = 0; i < data->MocapData->nMarkers; i++)
		{
			fprintf(fp, "\t%.5f\t%.5f\t%.5f", data->MocapData->Markers[i][0], data->MocapData->Markers[i][1], data->MocapData->Markers[i][2]);
		}
		fprintf(fp, "\n");
	}


	void _WriteFooter(FILE* fp)
	{
		fprintf(fp, "</Data>\n\n");
		fprintf(fp, "</MarkerSet>\n");
	}


	void resetClient()
	{
		int iSuccess;

		printf("\n\nre-setting Client\n\n.");

		iSuccess = g_pClient->Disconnect();
		if (iSuccess != 0)
			printf("error un-initting Client\n");

		iSuccess = g_pClient->Connect(g_connectParams);
		if (iSuccess != 0)
			printf("error re-initting Client\n");
	}


#ifndef _WIN32
	char getch()
	{
		char buf = 0;
		termios old = { 0 };

		fflush(stdout);

		if (tcgetattr(0, &old) < 0)
			perror("tcsetattr()");

		old.c_lflag &= ~ICANON;
		old.c_lflag &= ~ECHO;
		old.c_cc[VMIN] = 1;
		old.c_cc[VTIME] = 0;

		if (tcsetattr(0, TCSANOW, &old) < 0)
			perror("tcsetattr ICANON");

		if (read(0, &buf, 1) < 0)
			perror("read()");

		old.c_lflag |= ICANON;
		old.c_lflag |= ECHO;

		if (tcsetattr(0, TCSADRAIN, &old) < 0)
			perror("tcsetattr ~ICANON");

		//printf( "%c\n", buf );

		return buf;
	}
#endif
}