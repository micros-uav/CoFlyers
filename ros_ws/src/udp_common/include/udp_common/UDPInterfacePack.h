#ifndef _UDPINTERFACEPACK
#define _UDPINTERFACEPACK
#include"udp_common/UDPInterfaceBase.h"
#include"udp_common/data_description.h"
namespace udpPack{
    
    class UDPInterfacePack:public socketTools::UDPInterfaceBase
    {
    private:
        // socketTools::UDPInterfaceBase udpInterface;
        char buffRecv[msg::LEN_MAX_BUFFER]{};
        char buffSend[msg::LEN_MAX_BUFFER]{};
        // Pack msgData to buffSend
        int packMsg();
		// Unpack buffRecv to msgData
        int unpackMsg();
    public:
        void initialzePackageBuffer();
        void initialzeSocket(const char*ipLocal,int portLocal, const char*ipTarget,int portTarget,
        int sizeBuffSend,int sizeBuffRecv);

		//msg::package packRecv;
		//msg::package packSend;

        msg::package* packRecvPtr = nullptr;
        msg::package* packSendPtr = nullptr;
        UDPInterfacePack();
        UDPInterfacePack(uint8_t systemId, const char*ipLocal,int portLocal, const char*ipTarget,int portTarget,
        int sizeBuffSend,int sizeBuffRecv);
        ~UDPInterfacePack();

		// Call udpInterface.receiveData() and unpackMsg()
		int recvMsg();
		// Call packMsg() and udpInterface.sendData()
		ssize_t sendMsg();
		// 

        void set_package_send_common(const uint8_t componentId,const uint8_t messageId);
		void set_package_send_check_num();
		void set_package_data(const float*msgData,const uint8_t componentId,const uint8_t messageId);
		
  //  /// <summary>
  //  /// Determine the length of msgData according to componentId and messageId
  //  /// related to the platform you use.
  //  /// The function is user-defined and you can rewrite it.
  //  /// </summary>
  //  /// <param name=""></param>
  //      static int getMsgNum(uint8_t componentId,uint8_t messageId);
		//// Check sum
		//static uint16_t get_check_num(const msg::package& myPackage);
    };

}
#endif