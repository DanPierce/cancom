/*
 * 	controlAllegroHand.cpp
 *
 *  Created on: 		Nov 15, 2012
 *  Added to Project: 	Jan 17, 2013
 *  Author: 			Seungsu Kim & Alex Alspach
 */

#include "controlAllegroHand.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "ros/ros.h"
#include <string>


using namespace std;


controlAllegroHand::controlAllegroHand()
{
	radar_pub_ = nh_.advertise<can_msgs::RadarData>("radar", 10);
	prevPostID = 64;
}


controlAllegroHand::~controlAllegroHand()
{
	//PRINT_INFO("Setting System OFF");
	ROS_INFO("Setting System OFF");
	//_writeDeviceMsg(ID_CMD_SET_SYSTEM_OFF, ID_DEVICE_MAIN, ID_COMMON);
	usleep(10000);

	if(CAN_Close(CanHandle))
	{
		//PRINT_INFO("Error in CAN_Close()");
		ROS_ERROR("Error in CAN_Close()");
	}
	ros::shutdown();
}

void controlAllegroHand::init(int mode)
{

	
	unsigned char data[8];
	int ret;
	TPCANRdMsg lmsg;

	//PRINT_INFO("Opening CAN device");
	ROS_INFO("CAN: Opening device");
	

	CanHandle = LINUX_CAN_Open("/dev/pcan32", O_RDWR);
	if (!CanHandle)
	{
		//PRINT_INFO("Error in CAN_Open()");
		ROS_ERROR("CAN: Error in CAN_Open()");
	}

	char txt[VERSIONSTRING_LEN];
	ret = CAN_VersionInfo(CanHandle, txt);
	if (!ret)
	{
		//PRINT_INFO(txt);
		ROS_INFO("CAN: %s", txt);
	}
	else {
		//PRINT_INFO("Error getting CAN_VersionInfo()");
		ROS_ERROR("CAN: Error in CAN_VersionInfo()");
	}

	//PRINT_INFO("Initializing CAN device");
	ROS_INFO("CAN: Initializing device");
	// init to an user defined bit rate
	ret = CAN_Init(CanHandle, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
	if (ret)
	{
		//PRINT_INFO("Error in CAN_Init()");
		ROS_ERROR("CAN: Error in CAN_Init()");
	}

	//PRINT_INFO("Clear the can buffer");
	ROS_INFO("CAN: Clearing the CAN buffer");
	for(int i=0; i<100; i++){
		LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000); // polding
	}

	//PRINT_INFO("System off");
	usleep(500);

	for(int i=0; i<100; i++) ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 0);

	//PRINT_INFO("Setting joint query command");

	usleep(100);

	//cout << "started" << endl;
	initRadar();
	ROS_INFO("CAN: Communicating");
}

int controlAllegroHand::update(void)
{
	//unsigned char data[8];
	int ret;
	TPCANRdMsg lmsg;

	usleep(10);
	//_writeDevices();
	double q[4];
	char lID;
		ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000000);
		lID  = _parseCANMsg( lmsg.Msg, q);
	// ret=LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000000);
	// parseCANMsg( lmsg.Msg );
	// if(lmsg.ID)
	// 	ROS_INFO("READMESSAGE!!!!!");
	if(mEmergencyStop == true)
	{
		return 0;
	}
	else
	{
		return 0;
	}
}

int  controlAllegroHand::command(const short& cmd, const int& arg)
{
	return 0;
}




void controlAllegroHand::initRadar()
{
	double pwmDouble[DOF_JOINTS];
	short pwm[DOF_JOINTS];
	unsigned char data[8];

	TPCANMsg msg1;
	
	msg1.ID  = 0x4F1;
	msg1.MSGTYPE  = MSGTYPE_STANDARD;
	msg1.LEN  = 8;
	for(BYTE i=0; i<8; i++) msg1.DATA[i] = 0;

	msg1.DATA[6] = (unsigned char)(0xBF);

	//if(LINUX_CAN_Write_Timeout(CanHandle, &msg1, 0.0))
	if(CAN_Write(CanHandle, &msg1))
	{
		cout << "CAN communication error (write)" << endl;
		ROS_ERROR("CAN: Write error");
		mEmergencyStop = true;
	}else{
		ROS_INFO("RADAR INITIALIZED");
	}


	// data[0] = (unsigned char)(0x00);
	// data[1] = (unsigned char)(0x00);
	// data[2] = (unsigned char)(0x00);
	// data[3] = (unsigned char)(0x00);
	// data[4] = (unsigned char)(0x00);
	// data[5] = (unsigned char)(0x00);
	// data[6] = (unsigned char)(0xBF);
	// data[7] = (unsigned char)(0x00);

	// _writeDeviceMsg( (DWORD)(ID_CMD_SET_TORQUE_1 + findex), ID_DEVICE_MAIN,(BYTE)0x4F1, 8, data);
	usleep(10);

}

void controlAllegroHand::_writeDeviceMsg(DWORD command, DWORD from, DWORD to, BYTE len, unsigned char *data)
{
	TPCANMsg msg1;
	DWORD Txid;

	Txid = (command<<6) | (to <<3) | (from);
	msg1.ID  = Txid;
	msg1.MSGTYPE  = MSGTYPE_STANDARD;
	msg1.LEN  = len;
	for(BYTE i=0; i<8; i++) msg1.DATA[i] = 0;

	for(BYTE i=0; i<len; i++)
	{
		msg1.DATA[i] = data[i];
	}

	//if(LINUX_CAN_Write_Timeout(CanHandle, &msg1, 0.0))
	if(CAN_Write(CanHandle, &msg1))
	{
		cout << "CAN communication error (write)" << endl;
		ROS_ERROR("CAN: Write error");
		mEmergencyStop = true;
	}

}

void controlAllegroHand::_writeDeviceMsg(DWORD command, DWORD from,DWORD to)
{
	_writeDeviceMsg(command, from, to, 0, NULL);
}



char controlAllegroHand::_parseCANMsg(TPCANMsg read_msg,  double *values)
{
	unsigned char cmd, src, to;
	unsigned char len;
	unsigned char tmpdata[8];
	int tmppos[4];
	int lIndexBase;
	

	if ((read_msg.ID>=1280)&&(read_msg.ID<1344))
		parse_radar(read_msg);

	cmd = (unsigned char)( (read_msg.ID >> 6) & 0x1f );
	to  = (unsigned char)( (read_msg.ID >> 3) & 0x07 );
	src = (unsigned char)( read_msg.ID & 0x07 );
	len = (unsigned char)( read_msg.LEN );
	for(unsigned int nd=0; nd<len; nd++)
		tmpdata[nd] = read_msg.DATA[nd];
	switch (cmd)
	{
	case ID_CMD_QUERY_CONTROL_DATA:
		if (src >= ID_DEVICE_SUB_01 && src <= ID_DEVICE_SUB_04)
		{

			tmppos[0] = (int)(tmpdata[0] | (tmpdata[1] << 8));
			tmppos[1] = (int)(tmpdata[2] | (tmpdata[3] << 8));
			tmppos[2] = (int)(tmpdata[4] | (tmpdata[5] << 8));
			tmppos[3] = (int)(tmpdata[6] | (tmpdata[7] << 8));

			lIndexBase = 4*(src-ID_DEVICE_SUB_01);

			//values[0] = (double)mEncoderDirection[lIndexBase+0] * (double)(tmppos[0] - 32768 - mEncoderOffset[lIndexBase+0]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[1] = (double)mEncoderDirection[lIndexBase+1] * (double)(tmppos[1] - 32768 - mEncoderOffset[lIndexBase+1]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[2] = (double)mEncoderDirection[lIndexBase+2] * (double)(tmppos[2] - 32768 - mEncoderOffset[lIndexBase+2]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			//values[3] = (double)mEncoderDirection[lIndexBase+3] * (double)(tmppos[3] - 32768 - mEncoderOffset[lIndexBase+3]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[0] = ((double)mEncoderDirection[lIndexBase+0] *(double)tmppos[0] - 32768.0 - (double)mEncoderOffset[lIndexBase+0]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[1] = ((double)mEncoderDirection[lIndexBase+1] *(double)tmppos[1] - 32768.0 - (double)mEncoderOffset[lIndexBase+1]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[2] = ((double)mEncoderDirection[lIndexBase+2] *(double)tmppos[2] - 32768.0 - (double)mEncoderOffset[lIndexBase+2]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);
			values[3] = ((double)mEncoderDirection[lIndexBase+3] *(double)tmppos[3] - 32768.0 - (double)mEncoderOffset[lIndexBase+3]) * ( 333.3 / 65536.0 ) * ( M_PI/180.0);

			return src;

		}
		else
		{
			cout << "No subdevice match!" << endl;
			return -1;
		}

		break;
		//TODO: Implement this
	case ID_CMD_QUERY_STATE_DATA:
		return 0;
		break;
	default:
		//printf("unknown command %d, src %d, to %d, len %d \n", cmd, src, to, len);
		//ROS_WARN("unknown command %d, src %d, to %d, len %d", cmd, src, to, len);
		/*
		for(int nd=0; nd<len; nd++)
		{
			printf("%d \n ", tmpdata[nd]);
		}
		*/
		return -1;
		break;
	}

}

void controlAllegroHand::parse_radar(TPCANMsg myCANmsg){
	
	int track_id = myCANmsg.ID-1280;
	double dt;
	int aSignInt; int aval1; int aval2; int rval1; int rval2;
	int rrSignInt; int rrval1; int rrval2;
	int track_status;

	if (track_id < prevPostID){
	 	time_current = ros::Time::now().toSec();
	 	dt = time_current - time_prev;
		ROS_INFO("this: %g",dt);
		post_to_database(currentTracks);
		time_prev = time_current;
	}
	prevPostID = track_id;

	track_status = (int)((myCANmsg.DATA[1]&0xE0)>>5);

	if (track_status>0){
		// -------------------- Parse Angle Measurement --------------------- //
		aSignInt = ((myCANmsg.DATA[1]&0x10)<<5); //MSB: (DATA[1]&[00010000])*2^5
		aval1 = ((myCANmsg.DATA[1]&0xF)<<5); // (DATA[1]& [00001111])*2^5
		aval2 = (myCANmsg.DATA[2]&0xF8 >> 3);// (DATA[2]& [11111000])/2^3
		//currentTracks.angle[track_id] = (float)(0.1*(aval1 + aval2 - aSignInt)); // subtract MSB to sign value
		currentTracks.angle[track_id] = (float)(0.1*(aval1 + aval2 - aSignInt));
		// -------------------- Parse Range Measurement --------------------- //
		rval1 = ((myCANmsg.DATA[2]&0x7)<<8);// (DATA[2]&[00000111])*2^8
		rval2 = (myCANmsg.DATA[3]);
		currentTracks.range[track_id] = (float)(0.1*(rval1 + rval2));

		// ----------------- Parse Range Rate Measurement ------------------ //
		rrSignInt = ((myCANmsg.DATA[6]&0x20)<<8); //MSB: (DATA[6]&[00100000])*2^8
		rrval1 = ((myCANmsg.DATA[6]&0x1F)<<8); // (DATA[6]& [00011111])*2^8
		rrval2 = (myCANmsg.DATA[7]);//
		currentTracks.range_rate[track_id] = (float)(0.01*(rrval1 + rrval2 - rrSignInt));

		currentTracks.range_mode[track_id] = (int)((myCANmsg.DATA[6]&0xC0)>>6);

	// 	radarxyz.points.x = currentTracks.range[track_id]*sin(currentTracks.angle[track_id]*3.14/180);
	// 	radarxyz.points.x = currentTracks.range[track_id]*cos(currentTracks.angle[track_fredfred
	}else{
		currentTracks.angle[track_id] = 0;
		currentTracks.range[track_id] = 0;
		currentTracks.range_rate[track_id] = 0;
		currentTracks.range_mode[track_id] = 0;
	}

}

void controlAllegroHand::post_to_database(can_msgs::RadarData currentTracks){
	// if (currentTracks.range_mode[0]>0)
	// 	ROS_INFO("mode: %i",currentTracks.range_mode[0]); 

	radar_pub_.publish(currentTracks);
	return;
}