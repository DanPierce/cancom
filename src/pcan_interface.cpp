/**
* \file     pcan_interface.cpp
* \author   William Woodall
* \date     08-11-2010
* \brief    Source file for the PCAN USB-CAN device interface.
* 
*/

#include "pcan_interface.h"
#include "fcntl.h"

PCANInterface::PCANInterface() {
    #ifdef _WIN32
    msg = new TPCANMsg();
	msgTime = new TPCANTimestamp();
	pSetRcvEvent = NULL;
    #else
    msg = new TPCANRdMsg();
    #endif
}


PCANInterface::~PCANInterface() {
	Close_CAN();
}


CANMsg PCANInterface::ReadCAN() {
#ifdef _WIN32
	//CAN_Read(msg);
	CAN_ReadEx(msg,msgTime );
	//std::cout<<"Read"<<std::endl;
#else
	LINUX_CAN_Read(handle, msg);
#endif
	CANMsg can_msg;
#ifdef _WIN32
	if (msg->LEN>0 || msg->ID == 0x7FF)
	{
		can_msg.ID = msg->ID;
		can_msg.LEN = msg->LEN;
		for(int i = 0; i < msg->LEN; i++)
			can_msg.DATA[i] = msg->DATA[i];

		can_msg.STATUS = CAN_Status();
		can_msg.micros= msgTime->micros;
		can_msg.millis = msgTime->millis;
		can_msg.millis_overflow = msgTime->millis_overflow;
	}
	else//Oh snap there was no data and prob an error, populate struct with something to indicat this
	{
		can_msg.ID = -999;
		can_msg.LEN = -1;
		for(int i = 0; i < 8; i++)
			can_msg.DATA[i] = -1;
		can_msg.STATUS = CAN_Status();
	}
#else
    if (msg->Msg.LEN > 0 || msg->Msg.ID == 0x003) // why the or? 
    {
		can_msg.ID = msg->Msg.ID;
		can_msg.LEN = msg->Msg.LEN;
		for(int i = 0; i < msg->Msg.LEN; i++)
			can_msg.DATA[i] = msg->Msg.DATA[i];
		can_msg.STATUS = CAN_Status(handle);
	}
	else
	{
		can_msg.ID = -999;
		can_msg.LEN = -1;
		for(int i = 0; i < 8; i++)
			can_msg.DATA[i] = -1;
		can_msg.STATUS = LINUX_CAN_Extended_Status(handle, 0, 0);
	}
#endif
	return can_msg;
}

bool PCANInterface::Init_CAN(WORD baud, int message_type, DWORD from_low_id, DWORD to_high_id, const char *device){
	#ifdef _WIN32
		//Open a connection to the CAN bus
		//with a baud rate of baud, and a standard length msg
		CAN_Init(baud, message_type);
		//Rset the CAN Filters
		CAN_ResetFilter();
		//Setup CAN filter
		int err=CAN_MsgFilter(from_low_id,to_high_id,message_type);
		pSetRcvEvent = (ReceiveEvent)GetFunction("CAN_SetRcvEvent");
	#else
		// Open a connection to the CAN Bus
		handle = LINUX_CAN_Open(device, O_RDWR);
		// Initialize the CAN bus to 10000 Baud
		CAN_Init(handle, baud, message_type);
		// Reset the CAN Filters
		CAN_ResetFilter(handle);
		//Setup CAN filter
		int err=CAN_MsgFilter(handle,from_low_id,to_high_id,message_type);
	#endif
	printf("CAN Init Settings: Baud - %04X Message Type - %i Low ID - %i High ID - %i Device - %s\n", baud, message_type, from_low_id, to_high_id, device);
	if(err) {
	    std::cout << "Error on CAN Init, err: " << err << std::endl;
        return false;
	}
    return true;
}

void PCANInterface::Close_CAN(void)
{
	#ifdef _WIN32
		CAN_Close();
	#else
		// Cleanup and Close CAN connection
		CAN_Close(handle);
	#endif
}

void PCANInterface::WriteCAN(DWORD can_id, BYTE data_length, BYTE data[], BYTE msg_type=MSGTYPE_STANDARD) {
    // Prepare the CAN msg
    #ifdef _WIN32
    msg->MSGTYPE=msg_type;
    msg->ID=can_id;
    msg->LEN=data_length;
    for(int i = 0; i < data_length; i++)
        msg->DATA[i] = data[i];
    // Send the TPCANMsg to the CAN bus
	CAN_Write(msg);
	#else
	TPCANMsg trans_msg;
	trans_msg.MSGTYPE=msg_type;
    trans_msg.ID=can_id;
    trans_msg.LEN=data_length;
    for(int i = 0; i < data_length; i++)
        trans_msg.DATA[i] = data[i];
	// Send the TPCANMsg to the CAN bus
	CAN_Write(handle, &trans_msg);
	#endif
}

void PCANInterface::WriteCANMsg(TPCANMsg message) {
	#ifdef _WIN32
		// Send the TPCANMsg to the CAN bus
		CAN_Write(&message);
	#else
		// Send the TPCANMsg to the CAN bus
		CAN_Write(handle, &message);
	#endif
}
#ifdef _WIN32
CANResult PCANInterface::SetRcvEvent(HANDLE hEvent)
{
	
	// Function CAN_SetRcvEvent/CAN2_SetRcvEvent is called
	//
	return (CANResult)pSetRcvEvent(hEvent);  
}

// Gets the address of a given function name in a loaded DLL
//
FARPROC PCANInterface::GetFunction(char* strName)
{
	// There is no DLL loaded
	//


	// Gets the address of the given function in the loeaded DLL
	//
	hCANLightDll = LoadLibrary("PCAN_USB");
	return GetProcAddress(hCANLightDll, strName);
}
#endif