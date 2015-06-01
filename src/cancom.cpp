/*
 * 	controlAllegroHand.cpp
 *
 *  Created on: 		Nov 2013
 *  Author: 			Dan Pierce
 */

#include "cancom.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include "ros/ros.h"
#include <string>

using namespace std;

Cancom::Cancom()
{
	boost::mutex::scoped_lock lockReading(readingMutex);
	bReading=false;
	lockReading.unlock();
	bStartReading=false;

	is_init=false;
	ros::Time::init();
}


Cancom::~Cancom()
{
	ROS_INFO("Setting System OFF");
	usleep(10000);

	if(CAN_Close(CanHandle))
	{
		ROS_ERROR("Error in CAN_Close()");
	}
	ros::shutdown();
}

bool Cancom::init(int sensor_type, string port_)
{
	TPCANRdMsg lmsg;
	int ret;
	ROS_INFO("CAN: Opening device");
// port_.c_str()
	CanHandle = LINUX_CAN_Open(port_.c_str(), O_RDWR);
	if (!CanHandle)
	{
		ROS_ERROR("CAN: Error in CAN_Open()");
		return false;
	}

	char txt[VERSIONSTRING_LEN];
	ret = CAN_VersionInfo(CanHandle, txt);
	if (!ret)
	{
		ROS_INFO("CAN: %s", txt);
	}
	else {
		ROS_ERROR("CAN: Error in CAN_VersionInfo()");
	}

	ROS_INFO("CAN: Initializing device");
	// init to an user defined bit rate
	ret = CAN_Init(CanHandle, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
	if (ret)
	{
		ROS_ERROR("CAN: Error in CAN_Init()");
		return false;
	}

	ROS_INFO("CAN: Clearing the CAN buffer");
	for(int i=0; i<100; i++){
		LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000); // polding
	}

	usleep(500);

	if(sensor_type==0){
		if(!initRadar()) return false;
		// Initialize Point Clouds
		for (int ii=0;ii<64;ii++){
			Tracks.points[ii].x = 0.0; 
			Tracks.points[ii].y = 0.0;
			Tracks.points[ii].z = 0.0;
		}
	}

	is_init=true;
	ROS_INFO("CAN: Communicating");

	StartReading();

	return true;
}

bool Cancom::StartReading()
{
	if (bReading)
	{
		return false;
	}
	if(CAN_Status(CanHandle)>=0){
		bStartReading=true; // loop variable
		// start thread to listen for incoming com messages
		mReadThread = boost::shared_ptr<boost::thread > (new boost::thread(boost::bind(&Cancom::ReadThread, this)));
		// wait for read thread to start
		boost::mutex::scoped_lock lockReading(readingMutex); // get lock on bReading variable
		while (!bReading)
			readingCond.wait(lockReading);
		lockReading.unlock();
		ROS_INFO("READING STARTED");
	}
	else
	{
		boost::mutex::scoped_lock lockReading(readingMutex);
		bReading=false;
		lockReading.unlock();
		readingCond.notify_all();
	}

	return bReading;
}

void Cancom::ReadThread()
{
	int ret;
	TPCANRdMsg lmsg;
	// started reading
	boost::mutex::scoped_lock lockReading(readingMutex);
	bReading=true;
	lockReading.unlock();
	readingCond.notify_all();

	while (bStartReading)
	{
		ret = LINUX_CAN_Read_Timeout(CanHandle, &lmsg, 1000000);
		parse_can( lmsg);
	}

	// stopped reading
	lockReading.lock();
	bReading=false;
	lockReading.unlock();
	readingCond.notify_all();
}

bool Cancom::initRadar()
{

	TPCANMsg msg1;
	
	msg1.ID  = 0x4F1;
	msg1.MSGTYPE  = MSGTYPE_STANDARD;
	msg1.LEN  = 8;
	for(BYTE i=0; i<8; i++) msg1.DATA[i] = 0;

	msg1.DATA[6] = (unsigned char)(0xBF);

	if(CAN_Write(CanHandle, &msg1))
	{
		ROS_ERROR("ERROR WRITING RADIATION MSG");
		return false;
	}else{
		usleep(10);
		ROS_INFO("RADAR INITIALIZED");
		return true;
	}
}

void Cancom::parse_can(TPCANRdMsg read_msg_full){
	TPCANMsg read_msg = read_msg_full.Msg;

	int signInt; 
	int val1; 
	int val2; 
	
	if (is_init){
		if ((read_msg.ID>=1280)&&(read_msg.ID<1344)){

			int track_id = read_msg.ID-1280;
			int track_status = (int)((read_msg.DATA[1]&0xE0)>>5);

			
			currentTracks.header.stamp = ros::Time::now();

			currentTracks.track_status[track_id] = track_status;
			currentTracks.can_millis[track_id] = read_msg_full.dwTime;
			currentTracks.can_micros[track_id] = read_msg_full.wUsec;
			if (track_status>0){
				// -------------------- Parse Angle Measurement --------------------- //
				signInt = ((read_msg.DATA[1]&0x10)<<5); //MSB: (DATA[1]&[00010000])*2^5
				val1 = ((read_msg.DATA[1]&0xF)<<5); // (DATA[1]& [00001111])*2^5
				val2 = (read_msg.DATA[2]&0xF8 >> 3);// (DATA[2]& [11111000])/2^3
				currentTracks.angle[track_id] = (float)(0.1*(val1 + val2 - signInt));
				// -------------------- Parse Range Measurement --------------------- //
				val1 = ((read_msg.DATA[2]&0x7)<<8);// (DATA[2]&[00000111])*2^8
				val2 = (read_msg.DATA[3]);
				currentTracks.range[track_id] = (float)(0.1*(val1 + val2));
				// ----------------- Parse Range Rate Measurement ------------------ //
				signInt = ((read_msg.DATA[6]&0x20)<<8); //MSB: (DATA[6]&[00100000])*2^8
				val1 = ((read_msg.DATA[6]&0x1F)<<8); // (DATA[6]& [00011111])*2^8
				val2 = (read_msg.DATA[7]);//
				currentTracks.range_rate[track_id] = (float)(0.01*(val1 + val2 - signInt));

				currentTracks.range_mode[track_id] = (int)((read_msg.DATA[6]&0xC0)>>6);

				// if(abs(currentTracks.angle[track_id])<5){
					//ROS_INFO("RANGE: %f, RANGERATE: %f",currentTracks.range[track_id],currentTracks.range_rate[track_id]);
				// }

				Tracks.points[track_id].x = currentTracks.range[track_id]*cos(currentTracks.angle[track_id]);
				Tracks.points[track_id].y = currentTracks.range[track_id]*sin(currentTracks.angle[track_id]);
				Tracks.points[track_id].z = 0.0;

				geometry_msgs::TransformStamped point_trans;
				point_trans.header.stamp = ros::Time::now();
				point_trans.header.frame_id = "radar";
				// point_trans.child_frame_id = base_link_frame_id;

				point_trans.transform.translation.x = Tracks.points[track_id].x;
				point_trans.transform.translation.y = Tracks.points[track_id].y;
				point_trans.transform.translation.z = 0.0;
  				geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0 );

				point_trans.transform.rotation = q;

				tf_broadcaster.sendTransform(point_trans);


			}else{
				currentTracks.angle[track_id] = 0;
				currentTracks.range[track_id] = 0;
				currentTracks.range_rate[track_id] = 0;
				currentTracks.range_mode[track_id] = 0;
			}
		}
	}

	switch (read_msg.ID)
	{
		case 1249:
			// currentTracks.raw_data_mode = (int)((read_msg.DATA[1]&0x8)>>3);
			// currentTracks.grouping_mode = (int)((read_msg.DATA[4]&0x3));
			//ROS_INFO("raw_data_mode = %i",currentTracks.raw_data_mode);
			break;
		case 1344:
			// currentTracks.group_id = (int)(read_msg.DATA[0]&0xF);
			break;
		case 1537:
			// val1 = (read_msg.DATA[0]<<8);
			// val2 = (read_msg.DATA[1]);
			// groupscanindex = (int)((val1+val2));
			break;
		case 1539:
			// val1 = (read_msg.DATA[1]<<8);
			// val2 = (read_msg.DATA[2]);
			// currentTracks.filtered_range_int = (int)((val1+val2));
			// val1 = (read_msg.DATA[5]);
			// currentTracks.filtered_angle_int = (int)((val1));
			// val1 = (read_msg.DATA[0]);
			// currentTracks.group_id =  (int)((val1));
			break;
// -------------------------------- TRUCK MESSAGES ------------------------------------------------- //
		case 640:
			currentTruckMsg.header.stamp = ros::Time::now();
			currentTruckMsg.wheel_speed4 = (int)(read_msg.DATA[4]*256 + abs(read_msg.DATA[5]));
			break;
		case 644:
			currentTruckMsg.header.stamp = ros::Time::now();
			currentTruckMsg.wheel_speed3 = (int)(read_msg.DATA[4]*256 + abs(read_msg.DATA[5]));
			break;
		case 852:
			currentTruckMsg.header.stamp = ros::Time::now();
			currentTruckMsg.wheel_speed1 = (int)(read_msg.DATA[0]*256 + abs(read_msg.DATA[1]));
			break;
		case 853:
			currentTruckMsg.header.stamp = ros::Time::now();
			currentTruckMsg.wheel_speed2 = (int)(read_msg.DATA[0]*256 + abs(read_msg.DATA[1]));
			break;

		case 217055488: //0x0CF00100 EBC1 Page 371
			currentTruckMsg.brake_pedal_position = (int)abs(read_msg.DATA[1]);
		
		case 217055744://0x0CF00200 ETC1 Page 371
			for(int i_byte=0;i_byte<8;i_byte++)
				currentTruckMsg.ETC1[i_byte] = (int)abs(read_msg.DATA[i_byte]);
			
		case 217056512://0x0CF00500 ETC2 Page 372
			for(int i_byte=0;i_byte<8;i_byte++)
				currentTruckMsg.ETC2[i_byte] = (int)abs(read_msg.DATA[i_byte]);

		case 217056256: //0x0CF00400 EEC1 Page 372
			for(int i_byte=0;i_byte<8;i_byte++)
				currentTruckMsg.EEC1[i_byte] = (int)abs(read_msg.DATA[i_byte]);

			val1 = (int)abs(read_msg.DATA[3]);
			val2 = (int)abs(read_msg.DATA[4]);
			currentTruckMsg.engine_RPM = (float)(val1*0.125+val2*32);
			currentTruckMsg.percent_demand_torque = (int)abs(read_msg.DATA[1]) - 125;
			currentTruckMsg.percent_actual_torque = (int)abs(read_msg.DATA[2]) - 125;
			break;	
	}
}

can_msgs::RadarData Cancom::radarCanMsgCallback(){return currentTracks;};

sensor_msgs::PointCloud Cancom::radarPointCloudCallback(){return Tracks;};

can_msgs::TruckData Cancom::truckCanMsgCallback(){return currentTruckMsg;};