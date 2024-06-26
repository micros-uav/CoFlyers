/*  ********** Universal UDP Interface  ******** */
/*  Jialei Huang, 2023/02/15, SUN YAT-SEN University  */
/*  ********* huangjlei3@mail2.sysu.edu.cn ********** */
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  UniversalUDPInterface2
#define _WINSOCK_DEPRECATED_NO_WARNINGS 1

#include <iostream>
#include <string.h>

#if defined(__WIN32__)||defined(WIN32)||defined(_WIN32)
#include <winsock2.h>
#pragma comment(lib,"ws2_32.lib")
#define nonblockingsocket(s) {unsigned long ctl = 1;ioctlsocket( s, FIONBIO, &ctl );}

typedef int socklen_t;
typedef int ssize_t; 
#define MSG_NOSIGNAL 0
#define getMillisecond(tsPtr) {*tsPtr = GetTickCount();} 
#else

#include<netinet/in.h>
#include<unistd.h>
#include<math.h>
#include<arpa/inet.h>
#include<fcntl.h>
#include<sys/socket.h>
#include<sys/time.h>
typedef long SOCKET;
typedef sockaddr_in SOCKADDR_IN;
typedef sockaddr SOCKADDR;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define nonblockingsocket(s) { int flags = fcntl(s,F_GETFL,0);fcntl(s, F_SETFL, flags | O_NONBLOCK);}
#define closesocket(s) close(s)
#define getMillisecond(tsPtr) {struct timeval tp; gettimeofday(&tp,NULL); *tsPtr=((double)tp.tv_sec+(double)tp.tv_usec/CLOCKS_PER_SEC);} 
#endif



// #include"UDPInterfaceSimulink.h"
#include"UDPInterfacePack.h"

#include "simstruc.h"

#define getNumPtr(S) (double *)mxGetPr(ssGetSFcnParam(S,1))
//#define getGainsPtr(S) (double *)mxGetPr(ssGetSFcnParam(S,3))
#define getPortTargetPtr(S) (double *)mxGetPr(ssGetSFcnParam(S,2))
#define getPortLocalPtr(S) (double *)mxGetPr(ssGetSFcnParam(S,3))

#define IpTargetParam(S) ssGetSFcnParam(S,4)
#define getIpTargetMxArrayPtr(S) (double *)mxGetPr(IpTargetParam(S))

#define getIpLocalMxArrayPtr(S) ssGetSFcnParam(S,5)
#define getFlagSendPrmMXA(S) ssGetSFcnParam(S,6)
#define getFlagRecvPrmMXA(S) ssGetSFcnParam(S,7)
#define getInputPortWidthPtr(S) (double *)mxGetPr(ssGetSFcnParam(S,8))
#define getOutputPortWidthPtr(S) (double *)mxGetPr(ssGetSFcnParam(S,9))
// #define getComponentIdRecvPtr(S) (double *)mxGetPr(ssGetSFcnParam(S,9))
// #define getMessageIdRecvPtr(S) (double *)mxGetPr(ssGetSFcnParam(S,10))

#define numIpTarget mxGetN(IpTargetParam(S))

#define getFlagSend(S)                       \
    ( mxIsLogicalScalar(getFlagSendPrmMXA(S)) ? \
      mxIsLogicalScalarTrue(getFlagSendPrmMXA(S)) : \
      (mxGetScalar(getFlagSendPrmMXA(S)) != 0) )
#define getFlagRecv(S)                       \
    ( mxIsLogicalScalar(getFlagRecvPrmMXA(S)) ? \
      mxIsLogicalScalarTrue(getFlagRecvPrmMXA(S)) : \
      (mxGetScalar(getFlagRecvPrmMXA(S)) != 0) )
#define LEN_IP_STRING 16

udpPack::UDPInterfacePack *my_udp_interface_array = nullptr;
msg::package *pack_recv_array = nullptr;
msg::package *pack_send_array = nullptr;
bool flag_recv = true;
bool flag_send = false;

//Close socket and free memory 
void ClearUp(SimStruct *S){
    //Sleep(100);
    if (my_udp_interface_array != nullptr)
    {
        delete[] my_udp_interface_array;
    }
    my_udp_interface_array = nullptr;
}

unsigned int outputNumber = 16; 
unsigned int inputNumber = 6;

//void packe
/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */


/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS
/*
 * Check to make sure that each parameter is 1-d and positive
 */

static void mdlCheckParameters(SimStruct *S)
{
    // const int lenIP = 16;
    const double *sTimePtr = (double*)mxGetPr(ssGetSFcnParam(S,0));
    const double *numPtr = getNumPtr(S) ;
    const double *portLocalPtr = getPortLocalPtr(S);
    const double *portTargetPtr = getPortTargetPtr(S);
    int number = (int)floor((*numPtr)+0.5);
    int portLocal = (int)floor((*portLocalPtr)+0.5);
    int portTarget = (int)floor((*portTargetPtr)+0.5);
    //char ipLocal[LEN_IP_STRING];
    const double *ipTargetPtr = (double*)getIpTargetMxArrayPtr(S);
    const char *ipLocal = mxArrayToString(getIpLocalMxArrayPtr(S));
    //const char *ipLocal = getIpLocalPtr(S);
    
    //printf("%s %d %ld\n",ipLocal, strlen(ipLocal), inet_addr(ipLocal));
    //mxGetString(getIpLocalMxArrayPtr(S), ipLocal, LEN_IP_STRING);
    //mxGetString(getIpTargetMxArrayPtr(S), ipTarget, LEN_IP_STRING);
    const double *inputPortWidth = getInputPortWidthPtr(S);
    const double *outputPortWidth = getOutputPortWidthPtr(S);
    

    if (*sTimePtr <= 0)
        {ssSetErrorStatus(S,"The sample time must be positive."); return;}
    if (*numPtr <= 0)
        {ssSetErrorStatus(S,"The number of tello must be positive."); return;}
    if ( (portLocal < 1025) || (portLocal > 65535) )
        { ssSetErrorStatus(S,"The UDP port of local must be from 1025 to 65535"); return; }
    if ( (portTarget < 1025) || (portTarget > 65535) )
        { ssSetErrorStatus(S,"The UDP port of target must be from 1025 to 65535"); return; }
    if(strcmp(ipLocal,"255.255.255.255")!=0){
    if(inet_addr(ipLocal)==INADDR_NONE){
        //printf("Can not process the local IP address %s.",ipLocal);
        ssSetErrorStatus(S, "The format of the local IP address is wrong."); 
        return; 			
    }
    }
    if (numIpTarget != *numPtr)
    {
        ssSetErrorStatus(S, "The columns number of the target IP address must be equal to number."); 
    }
    for (int i = 0; i < numIpTarget; i++)
    {
        
        if ((ipTargetPtr[i]<1) || (ipTargetPtr[i]>255))
        { 
            printf("%lf\n",ipTargetPtr[i]);
            ssSetErrorStatus(S,"The ip address must be from 1 to 255"); 
            return; 
        }
        
    }
    
    // if(strcmp(ipTarget,"255.255.255.255")!=0){
    // if(inet_addr(ipTarget)==INADDR_NONE){
    //     //printf("Can not process the local IP address %s.",ipLocal);
    //     ssSetErrorStatus(S, "The format of the target IP address is wrong."); 
    //     return; 			
    // }
    // }
    if (*inputPortWidth < 0)
        {ssSetErrorStatus(S,"The width of input port must be positive or 0."); return;}
    if (*outputPortWidth < 0)
        {ssSetErrorStatus(S,"The width of output port must be positive or 0."); return;}
        

}


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 10);  /* Number of expected parameters */

    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    ssSetSFcnParamTunable(S, 0, 0);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    double * numPtr = getNumPtr(S);
    int number = (int)floor((*numPtr)+0.5);
    const double *inputPortWidth = getInputPortWidthPtr(S);
    const double *outputPortWidth = getOutputPortWidthPtr(S);
    
    inputNumber = (unsigned int)(*inputPortWidth+0.5);
    outputNumber = (unsigned int)(*outputPortWidth+0.5);
    
    flag_recv = getFlagRecv(S);
    flag_send = getFlagSend(S);
    if (flag_send){
        //checkCommandType(S,indCommandType,inputNumber);
        if (!ssSetNumInputPorts(S, 1)) return;
        ssSetInputPortWidth(S,0, inputNumber*number);            /*  intput port width              */
        ssSetInputPortDataType(S,0,SS_DOUBLE);            /*  intput port data type          */  
    }
    else{
        if (!ssSetNumInputPorts(S, 0)) return;
    }
    if (flag_recv){
        if (!ssSetNumOutputPorts(S, 1)) return; //
        ssSetOutputPortWidth(S,0, outputNumber*number);           /*  output port width              */
        ssSetOutputPortDataType(S,0,SS_DOUBLE);            /*  output port data type          */
    }
    else{
        if (!ssSetNumOutputPorts(S, 0)) return;
    }
    ssSetNumSampleTimes(S, 0);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0); // reserve element in the pointers vector
    ssSetNumModes(S, 0); // to store a C++ object
    ssSetNumNonsampledZCs(S, 0);

    ssSetOperatingPointCompliance(S, USE_CUSTOM_OPERATING_POINT);

    /* Set this S-function as runtime thread-safe for multicore execution */
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
    
    ssSetOptions(S, 
                SS_OPTION_WORKS_WITH_CODE_REUSE |
                SS_OPTION_EXCEPTION_FREE_CODE |
                SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
#if defined(__WIN32__)||defined(WIN32)||defined(_WIN32)
		WSADATA wsd;
		if (WSAStartup(MAKEWORD(2, 2), &wsd))
		{
			printf("WSAStartup failed!\n");
			exit(1);
		}
#endif
    //Get Parameters from S-Function model
    const double *numPtr = getNumPtr(S) ;
    const double *portLocalPtr = getPortLocalPtr(S);
    const double *portTargetPtr = getPortTargetPtr(S);
    int number = (int)floor((*numPtr)+0.5);
    int portLocal = (int)floor((*portLocalPtr)+0.5);
    int portTarget = (int)floor((*portTargetPtr)+0.5);
    //char ipLocal[LEN_IP_STRING];
    //mxGetString(getIpLocalMxArrayPtr(S), ipLocal, LEN_IP_STRING);
    const char *ipLocal = mxArrayToString(getIpLocalMxArrayPtr(S));
    const double *ipTargetPtr = (double*)getIpTargetMxArrayPtr(S);
    char ipTarget[LEN_IP_STRING]{};
    int dot_count = 0;
    int dot_index = 0;
    for (int i = 0; i < LEN_IP_STRING; i++)
    {
        ipTarget[i] = ipLocal[i];
        if (ipLocal[i] == '.')
        {
            dot_count++;
        }
        if (dot_count == 3)
        {
            dot_index = i;
            break;
        }   
    }

    // for (int i = 0; i < numIpTarget; i++)
    // {
    //     snprintf(&ipTarget[dot_index+1],3,"%d",(int)(ipTargetPtr[i] + 0.5));
    //     printf("%s\n",ipTarget);
    // }

    //mxGetString(getIpTargetMxArrayPtr(S), ipTarget, 200);

    //Initialze udp interface
    //local:  ipLocal, portLocal+i*2
    //target: ipLocal, portTarget+i*2+1
    my_udp_interface_array = new udpPack::UDPInterfacePack[number];
    pack_recv_array = new msg::package[number];
    pack_send_array = new msg::package[number];
    for (int i = 0; i < number; i++)
    {
        snprintf(&ipTarget[dot_index+1],4,"%d",(int)(ipTargetPtr[i] + 0.5));
        my_udp_interface_array[i].initialzeSocket(ipLocal,portLocal+i*2,ipTarget,portTarget+i*2+1,
		msg::LEN_MAX_BUFFER,msg::LEN_MAX_BUFFER);
        my_udp_interface_array[i].initSocket();
        my_udp_interface_array[i].packRecvPtr = &(pack_recv_array[i]);
        my_udp_interface_array[i].packSendPtr = &(pack_send_array[i]);
        my_udp_interface_array[i].packSendPtr->systemId = i;
        printf("%s,%d,%s,%d\n",ipLocal,portLocal+i*2,ipTarget,portTarget+i*2+1);
    }
    
    //printf("%d,%d\n",flag_recv,flag_send);
  }                                            // pointers vector
#endif /*  MDL_START */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    if (flag_recv)
    {
        double * numPtr = getNumPtr(S);
        int number = (int)floor((*numPtr)+0.5);

        void *out = ssGetOutputPortSignal(S,0);
        double *dataOut= (double*) out;
        uint8_t componentId = 0;
        uint8_t messageId = 0;
        float *msgData = nullptr;
        udpPack::UDPInterfacePack* my_udp_interface = my_udp_interface_array;
        for (int id = 0; id < number; id++)
        {
            int revlen = my_udp_interface->recvMsg();
            if(revlen > 0)
            {
                componentId = my_udp_interface->packRecvPtr->componentId;
                messageId = my_udp_interface->packRecvPtr->messageId;
                // if (componentId == *componentIdRecv && messageId == *messageIdRecv)
                // {
                    //out=ssGetOutputPortSignal(S,id);
                    //dataOut = (double*) out;
                *dataOut++ = (double)(my_udp_interface->packRecvPtr->systemId);
                *dataOut++ = (double)(my_udp_interface->packRecvPtr->numSequence);
                *dataOut++ = (double)(my_udp_interface->packRecvPtr->timeStamp);
                *dataOut++ = (double)(componentId);
                *dataOut++ = (double)(messageId);

                msgData = my_udp_interface->packRecvPtr->msgData;
                int num = msg::getMsgNum(componentId,messageId);
                for (unsigned int j = 0; j < num; j++)
                {
                    *dataOut++ = (double)*msgData++;
                }
                for (unsigned int j = 0; j < (outputNumber-5-num); j++)
                {
                    *dataOut++ =0;
                }
                
                // }
            }
            else
            {
                dataOut+=outputNumber;
            }
            my_udp_interface++;
        }
    }
}                                                


#define MDL_UPDATE
/* Function: mdlUpdate ======================================================
 * Abstract:
 *      xdot = Ax + Bu
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    if (flag_send)
    {
        double * numPtr = getNumPtr(S);
        int number = (int)floor((*numPtr)+0.5);
        double **u = (double**) ssGetInputPortSignalPtrs(S,0); // Input pointer

        udpPack::UDPInterfacePack* my_udp_interface = my_udp_interface_array;
        uint8_t componentId = 0;
        uint8_t messageId = 0;
        float*msgData = nullptr;
        for (int id = 0; id < number; id++)
        {
            // my_udp_interface = (sim::UDPInterfaceSimulink*)PWork[id];
            //u = (double**) ssGetInputPortSignalPtrs(S,id); //Get input datas of port id
            componentId = (uint8_t)*(*u++);
            messageId = (uint8_t)*(*u++);
            msgData = my_udp_interface->packSendPtr->msgData;
            for (unsigned int j = 2; j < inputNumber; j++)
            {
                *msgData = (float)*(*u++);
                msgData++;
            }
            my_udp_interface->set_package_send_common(componentId, messageId);
            // printf("%d\n",id);
            // printf("N0.%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%lf,%d\n",id,
            //     my_udp_interface->packSendPtr->start,
            //     my_udp_interface->packSendPtr->lenPayload,
            //     my_udp_interface->packSendPtr->numSequence,
            //     my_udp_interface->packSendPtr->systemId,
            //     my_udp_interface->packSendPtr->componentId,
            //     my_udp_interface->packSendPtr->messageId,
            //     my_udp_interface->packSendPtr->msgData[0],
            //     my_udp_interface->packSendPtr->msgData[1],
            //     my_udp_interface->packSendPtr->msgData[2],
            //     my_udp_interface->packSendPtr->msgData[3],
            //     my_udp_interface->packSendPtr->timeStamp,
            //     my_udp_interface->packSendPtr->checkNum);
            // printf("N0.%d,%f,%f,%f,%f\n",id,
            //     my_udp_interface->packSendPtr->msgData[0],
            //     my_udp_interface->packSendPtr->msgData[1],
            //     my_udp_interface->packSendPtr->msgData[2],
            //     my_udp_interface->packSendPtr->msgData[3]);
            my_udp_interface->sendMsg();
            my_udp_interface++;
        }

    }
    
}
/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    ClearUp(S);
}                                              // function
/*======================================================*
 * See sfuntmpl.doc for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

