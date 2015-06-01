/*
 * 	controlAllegroHand.h
 *
 *  Created on: 		Nov 15, 2012
 *  Added to Project: 	Jan 17, 2013
 *  Authors: 			Seungsu Kim & Ashwini Schukla
 */

#ifndef CONTROLALLEGROHAND_H_
#define CONTROLALLEGROHAND_H_

#include <list>
#include "BHand.h"
#include "allegroCANProtocol.h"
#include <libpcan.h>
#include <fcntl.h>
#include <string>
 #include "ros/ros.h"
#include <std_msgs/String.h>
 #include <sensor_msgs/PointCloud.h>
 #include <can_msgs/RadarData.h>

#define ALLEGRO_CONTROL_TIME_INTERVAL 0.003

#define PWM_LIMIT_ROLL 250.0*1.5
#define PWM_LIMIT_NEAR 450.0*1.5
#define PWM_LIMIT_MIDDLE 300.0*1.5
#define PWM_LIMIT_FAR 190.0*1.5

#define PWM_LIMIT_THUMB_ROLL 350.0*1.5
#define PWM_LIMIT_THUMB_NEAR 270.0*1.5
#define PWM_LIMIT_THUMB_MIDDLE 180.0*1.5
#define PWM_LIMIT_THUMB_FAR 180.0*1.5




class controlAllegroHand
{

public:
	controlAllegroHand();
	~controlAllegroHand();

	void init(int mode = 0);
	int update(void);
	int  command(const short& cmd, const int& arg = 0);

	void setTorque(double *torque);
//	void getJointInfo(double *position, double *torque);
	void getJointInfo(double *position);

	struct TrackArray
	{
		float range[64];
		float angle[64];
		float range_rate[64];
		//float width[64];
		int range_mode[64];
		int track_status[64];
		//int rawAng[64];
		int msgStatus;
	};

private:
	HANDLE CanHandle;
	//TPCANMsg read_msg;
	TPCANRdMsg read_msg;

	double curr_position[DOF_JOINTS];
	double curr_torque[DOF_JOINTS];
	double desired_position[DOF_JOINTS];
	double desired_torque[DOF_JOINTS];
	
	double hand_version;

	double mPWM_MAX[DOF_JOINTS];
	int    mEncoderOffset[DOF_JOINTS];
	int    mEncoderDirection[DOF_JOINTS];
	int    mMotorDirection[DOF_JOINTS];
	ros::NodeHandle nh_;
	ros::Publisher radar_pub_;
	can_msgs::RadarData currentTracks;
	sensor_msgs::PointCloud radarxyz;
	//std::string hand_version;
	double time_current;
	double time_prev;
	bool bPost;
	int prevPostID;
	volatile bool mEmergencyStop;

	void _readDevices();
	DWORD _readDevicesCAN(unsigned char& id, double *position );
	void initRadar();
	void _parseCANMsg();
	void parse_radar(TPCANMsg myCANmsg);
	void post_to_database(can_msgs::RadarData currentTracks);
	void _writeDeviceMsg(DWORD command, DWORD from, DWORD to, BYTE len, unsigned char *data);
	void _writeDeviceMsg(DWORD command, DWORD from, DWORD to);
	//void _writeDeviceMsg(unsigned long command, unsigned long from, unsigned long to, BYTE len, unsigned char *data);
	//void _writeDeviceMsg(unsigned long command, unsigned long from, unsigned long to);
	char _parseCANMsg(TPCANMsg read_msg,  double *values);
	//char _parseCANMsg(TPCANMsg &read_msg,  double *values);

};


#endif /* CONTROLALLEGROHAND_H_ */
