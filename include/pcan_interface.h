/**
* \file     pcan_interface.h
* \author   William Woodall
* \date     08-11-2010
* \brief    Header file for the PCAN USB-CAN device interface.
* 
*/

#ifndef PCAN_INTERFACE__H
#define PCAN_INTERFACE__H

#include <stdio.h>
#include <iostream>

// The windows and linux drivers have different library names
//#define OS_WINDOWS
#ifdef _WIN32
	#include "windows.h"
    #include <pcan_usb.h>
#else
    #include <libpcan.h>
#endif

typedef struct 
{
  DWORD ID;             // 11/29 bit code
  BYTE  MSGTYPE;        // bits of MSGTYPE_*
  BYTE  LEN;            // count of data bytes (0..8)
  BYTE  DATA[8];        // data bytes, up to 8
  int   STATUS;
  #ifdef _WIN32
  DWORD millis;          // Base-value: milliseconds: 0.. 2^32-1
    WORD  millis_overflow; // Roll-arounds of millis
    WORD  micros;          // Microseconds: 0..999
  #endif
} CANMsg;               // Contains data of a CAN msg


#ifdef _WIN32
enum CANResult
{
    ERR_OK				= 0x0000,	// No error
    ERR_XMTFULL			= 0x0001,   // Send buffer of the Controller ist full
    ERR_OVERRUN			= 0x0002,   // CAN-Controller was read to late
    ERR_BUSLIGHT		= 0x0004,   // Bus error: an Error count reached the limit
    ERR_BUSHEAVY		= 0x0008,   // Bus error: an Error count reached the limit
    ERR_BUSOFF			= 0x0010,   // Bus error: CAN_Controller went to 'Bus-Off'
    ERR_QRCVEMPTY		= 0x0020,   // RcvQueue is empty
    ERR_QOVERRUN		= 0x0040,   // RcvQueue was read to late
    ERR_QXMTFULL		= 0x0080,   // Send queue is full
    ERR_REGTEST			= 0x0100,   // RegisterTest of the 82C200/SJA1000 failed
    ERR_NOVXD			= 0x0200,   // Problem with Localization of the VxD    
    ERR_ILLHW			= 0x1400,   // Invalid Hardware handle
    ERR_RESOURCE		= 0x2000,   // Not generatably Resource (FIFO, Client, Timeout)
    ERR_PARMTYP			= 0x4000,   // Parameter not permitted
    ERR_PARMVAL			= 0x8000,   // Invalid Parameter value
	ERRMASK_ILLHANDLE	= 0x1C00,   // Mask for all Handle errors
    ERR_ANYBUSERR		= ERR_BUSLIGHT | ERR_BUSHEAVY | ERR_BUSOFF, // All others error status <> 0 please ask by PEAK ......intern Driver errors.....
    ERR_NO_DLL			= 0xFFFFFFFF// A Dll could not be loaded or a function was not found into the Dll
};


typedef DWORD (__stdcall *ReceiveEvent)(HANDLE);	
#endif
//DECLARE_HANDLE(HINSTANCE);
// SetRcvEvent
/**
* \class    PCANInterface
* \brief    Class containing an interface and listener to the PCAN USB to CAN converter.
* 
*/
class PCANInterface {
public:
    PCANInterface();
    virtual ~PCANInterface();
    
	
	bool Init_CAN(WORD baud=CAN_BAUD_10K, int message_type=MSGTYPE_STANDARD, DWORD from_low_id=0x000, DWORD to_high_id=0x7FF, const char *device=NULL);
	void Close_CAN();
    CANMsg ReadCAN();
    void WriteCAN(DWORD can_id, BYTE data_length, BYTE data[], BYTE msg_type);
    void WriteCANMsg(TPCANMsg message);
    #ifdef _WIN32
	   CANResult SetRcvEvent(HANDLE hEvent);	
    #endif
protected:
    HANDLE handle;
    #ifdef _WIN32
    HINSTANCE hCANLightDll;
    TPCANMsg *msg;
    TPCANTimestamp*  msgTime;
    #else
    TPCANRdMsg *msg;
    #endif
    
#ifdef _WIN32
private:
	ReceiveEvent pSetRcvEvent;	
	FARPROC GetFunction(LPSTR szName);
#endif
};

#endif

