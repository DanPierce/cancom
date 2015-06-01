
/**************************************************************************************
 *
 *  parse_can.h
 *
 *  Dan Pierce
 *  November 2013			 
 *
 *********************************************************************************************/

//This code is setup to take in a single message from the can bus, parse the message,
//and return the parsed message as a struct

 #ifndef _PARSE_CAN_
 #define _PARSE_CAN_

 #include "cancom.h"


parsed_CAN_message parse_can( CANMsg myCANmsg, bool post_unkown_message=false){

    parsed_CAN_message myparsedCANmsg;
	std::stringstream ss;
	int signInt, val1, val2;
	// msg.getAt - returns specific byte of message
	if ((myCANmsg.ID >= 1280)&&(myCANmsg.ID < 1344))
	{
		myparsedCANmsg.NUM_MSG		= 4;
		myparsedCANmsg.ID			= myCANmsg.ID;
		myparsedCANmsg.LEN			= myCANmsg.LEN;
		myparsedCANmsg.STATUS		= (int)((myCANmsg.DATA[1]&0xE0)>>5);
		signInt = ((myCANmsg.DATA[1]&0x10)<<5); //MSB: (DATA[1]&[00010000])*2^5
		val1 = ((myCANmsg.DATA[1]&0xF)<<5); // (DATA[1]& [00001111])*2^5
		val2 = (myCANmsg.DATA[2]&0xF8 >> 3);// (DATA[2]& [11111000])/2^3
		ss << (int)(val1 + val2 - signInt);
		myparsedCANmsg.DATA         = ss.str();
		val1 = ((myCANmsg.DATA[2]&0x7)<<8);// (DATA[2]&[00000111])*2^8
		val2 = (myCANmsg.DATA[3]);
		ss << (int)(val1 + val2);
		myparsedCANmsg.DATA2        = ss.str();
		signInt = ((myCANmsg.DATA[6]&0x20)<<8); //MSB: (DATA[6]&[00100000])*2^8
		val1 = ((myCANmsg.DATA[6]&0x1F)<<8); // (DATA[6]& [00011111])*2^8
		val2 = (myCANmsg.DATA[7]);//
		ss << (int)(val1 + val2 - signInt);
		myparsedCANmsg.DATA3        = ss.str();
		ss << (int)((myCANmsg.DATA[6]&0xC0)>>6);
		myparsedCANmsg.DATA4        = ss.str(); 
		myparsedCANmsg.message_name = "Angle";
		myparsedCANmsg.message_name2= "Range";
		myparsedCANmsg.message_name3= "RangeRate";
		myparsedCANmsg.message_name4= "RangeMode";
	}
	else
	{
		myparsedCANmsg.ID			= -999;
		myparsedCANmsg.LEN			= 0;
		myparsedCANmsg.STATUS		= -999;
		myparsedCANmsg.DATA         = "IGNORE";
		myparsedCANmsg.message_name = "DON'T POST";
	}


	// 	switch(myCANmsg.m_iID)
	// 	{

	// 		//case 217056000:
	// 		//	int engine_rpm;

	// 		//	//engine_rpm=(myCANmsg.DATA[1]<<8)+ abs(myCANmsg.getAt(0)); //shift [1]byte by 8 bits and add LSB
	// 		//	//if(myCANmsg.DATA[1]>127)
	// 		//	//	steer_angle = steer_angle - 65536;  //take two's complement

	// 		//	engine_rpm = ((int)abs(myCANmsg.getAt(0))*0.125)+((int)abs(myCANmsg.DATA[1])*32)

	// 		//	myparsedCANmsg.NUM_MSG		= 1;
	// 		//	myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		//	myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		//	myparsedCANmsg.STATUS		= myCANmsg.STATUS;
	// 		//	myparsedCANmsg.DATA         = stringUtils::to_string( steer_angle );
	// 		//	myparsedCANmsg.message_name = "SteerAngle";
	// 		//	break;
	// 	/*
	// 		case 640:
	// 			myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 			myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 			myparsedCANmsg.STATUS		= myCANmsg.STATUS;
	// 			myparsedCANmsg.DATA         = stringUtils::to_string( (int)(myCANmsg.DATA[4]*256 + abs(myCANmsg.DATA[5])) );
	// 			myparsedCANmsg.message_name = "Wheel_Speed4";
	// 			break;


	// 		case 644:
	// 			myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 			myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 			myparsedCANmsg.STATUS		= myCANmsg.STATUS;
	// 			myparsedCANmsg.DATA         = stringUtils::to_string( (int)(myCANmsg.DATA[4]*256 + abs(myCANmsg.DATA[5])) );
	// 			myparsedCANmsg.message_name = "Wheel_Speed3";
	// 			break;

	// 		case 852:
	// 			myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 			myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 			myparsedCANmsg.STATUS		= myCANmsg.STATUS;
	// 			myparsedCANmsg.DATA         = stringUtils::to_string( (int)(myCANmsg.getAt(0)*256 + abs(myCANmsg.DATA[1])) );
	// 			myparsedCANmsg.message_name = "Wheel_Speed1";
	// 			break;


	// 		case 853:
	// 			myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 			myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 			myparsedCANmsg.STATUS		= myCANmsg.STATUS;
	// 			myparsedCANmsg.DATA         = stringUtils::to_string( (int)(myCANmsg.getAt(0)*256 + abs(myCANmsg.DATA[1])) );
	// 			myparsedCANmsg.message_name = "Wheel_Speed2";
	// 			break;
	// 	*/
				
					

	// 		// case 217056256: //0x0CF00400 EEC1 Page 372
	// 		// 	myparsedCANmsg.NUM_MSG		= 3;
	// 		// 	myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		// 	myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		// 	myparsedCANmsg.STATUS		= myCANmsg.STATUS;                                                                 //MSB
	// 		// 	myparsedCANmsg.DATA         = stringUtils::to_string( ((int)abs(myCANmsg.DATA[3])*0.125)+((int)abs(myCANmsg.DATA[4])*32) );
	// 		// 	myparsedCANmsg.DATA2        = stringUtils::to_string( (int)abs(myCANmsg.DATA[1]) - 125 );
	// 		// 	myparsedCANmsg.DATA3        = stringUtils::to_string( (int)abs(myCANmsg.DATA[2]) - 125 ); 
	// 		// 	myparsedCANmsg.message_name = "EngineRPM";
	// 		// 	myparsedCANmsg.message_name2= "Driver_Demand_Perc_Torque";
	// 		// 	myparsedCANmsg.message_name3= "Actual_Engine_Perc_Torque";
	// 		// 	break;

	// 		// case 419361024: //0x018FEF100 Cruise Control / Vehicle Speed Page 382
	// 		// 	myparsedCANmsg.NUM_MSG		= 1;
	// 		// 	myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		// 	myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		// 	myparsedCANmsg.STATUS		= myCANmsg.STATUS;
	// 		// 	myparsedCANmsg.DATA         = stringUtils::to_string( ((int)abs(myCANmsg.DATA[1])*1/412)+((int)abs(myCANmsg.DATA[2])*0.62) );
	// 		// 	myparsedCANmsg.message_name = "Wheel_based_vehicle_speed_mph";
	// 		// 	break;

	// 		// case 217056000: //0CF00300 EEC2 Page 372
	// 		// 	myparsedCANmsg.NUM_MSG		= 2;
	// 		// 	myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		// 	myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		// 	myparsedCANmsg.STATUS		= myCANmsg.STATUS;                                                                 
	// 		// 	myparsedCANmsg.DATA         = stringUtils::to_string( ((int)abs(myCANmsg.DATA[1]))*0.4 );
	// 		// 	myparsedCANmsg.DATA2        = stringUtils::to_string(  (int)(myCANmsg.DATA[2]) );
	// 		// 	myparsedCANmsg.message_name = "Accelerator_pedal_position";
	// 		// 	myparsedCANmsg.message_name2= "Perc_load_at_current_speed";
	// 		// 	break;

	// 		// case 419355904: //18FEDD00 TruboCharger Page 373
	// 		// 	myparsedCANmsg.NUM_MSG		= 1;
	// 		// 	myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		// 	myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		// 	myparsedCANmsg.STATUS		= myCANmsg.STATUS;
	// 		// 	myparsedCANmsg.DATA         = stringUtils::to_string( (((int)abs(myCANmsg.DATA[1]))+((int)abs(myCANmsg.DATA[2])*256))*4 );
	// 		// 	myparsedCANmsg.message_name = "TurboCharger1_speed";
	// 		// 	break;

	// 		// case 419360256: //18FEEE00 Engine Temperature Page 380

	// 		// 	int engine_coolant_temp;
					
	// 		// 	engine_coolant_temp = myCANmsg.getAt(0); //shift [1]byte by 8 bits and add LSB
	// 		// 	if(myCANmsg.getAt(0)>127)
	// 		// 		engine_coolant_temp = engine_coolant_temp - 128;  //take two's complement


	// 		// 	int engine_intercooler_temp;
				
	// 		// 	engine_intercooler_temp = myCANmsg.DATA[6]; //shift [1]byte by 8 bits and add LSB
	// 		// 	if(myCANmsg.DATA[6]>127)
	// 		// 	engine_intercooler_temp = engine_intercooler_temp - 128;  //take two's complement


	// 		// 	myparsedCANmsg.NUM_MSG		= 2;
	// 		// 	myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		// 	myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		// 	myparsedCANmsg.STATUS		= myCANmsg.STATUS;                                                                 
	// 		// 	myparsedCANmsg.DATA         = stringUtils::to_string(engine_coolant_temp);
	// 		// 	myparsedCANmsg.DATA2        = stringUtils::to_string(engine_intercooler_temp);
	// 		// 	myparsedCANmsg.message_name = "Engine_coolant_temp";
	// 		// 	myparsedCANmsg.message_name2= "Engine_intercooler_temp";
	// 		// break;

	// 		// case 419360768: //18FEF000 Power Takeoff INFO page 381
	// 	 //    break;
	// 		// 	myparsedCANmsg.NUM_MSG		= 1;
	// 		// 	myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		// 	myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		// 	myparsedCANmsg.STATUS		= myCANmsg.STATUS;                                                                 
	// 		// 	myparsedCANmsg.DATA         = stringUtils::to_string( (((myCANmsg.DATA[1])))*0.125 + (((myCANmsg.DATA[2])))*0.125);
	// 		// 	myparsedCANmsg.message_name = "Power_takeoff_speed";
	// 		// break;

	// 		// case 419361280:// Fuel Economy Info page 382

	// 		// 	double fuel_rate;
	// 		// 	double instant_fuel_economy;
	// 		// 	double avg_fuel_economy;
	// 		// 	double throttle_posn;

	// 		// 	fuel_rate =            (((int)abs(myCANmsg.DATA[1]))*256 + ((int)abs(myCANmsg.getAt(0))))*0.05;
	// 		// 	instant_fuel_economy = (((int)abs(myCANmsg.DATA[3]))*256 + ((int)abs(myCANmsg.DATA[2])))/512;
	// 		// 	avg_fuel_economy =     (((int)abs(myCANmsg.DATA[5]))*256 + ((int)abs(myCANmsg.DATA[4])))/512;
	// 		// 	throttle_posn =        ((unsigned int)(myCANmsg.DATA[6]))*0.4;
				
	// 		// 	myparsedCANmsg.NUM_MSG		= 4;
	// 		// 	myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		// 	myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		// 	myparsedCANmsg.STATUS		= myCANmsg.STATUS;                                                                 
	// 		// 	myparsedCANmsg.DATA         = stringUtils::to_string(fuel_rate);
	// 		// 	myparsedCANmsg.DATA2        = stringUtils::to_string(instant_fuel_economy);
	// 		// 	myparsedCANmsg.DATA3        = stringUtils::to_string(avg_fuel_economy);
	// 		// 	myparsedCANmsg.DATA4        = stringUtils::to_string(throttle_posn);
	// 		// 	myparsedCANmsg.message_name = "Fule_rate";
	// 		// 	myparsedCANmsg.message_name2= "Instantaneous_fuel_economy";
	// 		// 	myparsedCANmsg.message_name3= "Average_fuel_economy";
	// 		// 	myparsedCANmsg.message_name4= "Throttle_position";

	// 		// break;

	// 		// default:
	// 		// 	if( post_unkown_message )
	// 		// 	{
	// 		// 		myparsedCANmsg.ID			= myCANmsg.m_iID;
	// 		// 		myparsedCANmsg.LEN			= myCANmsg.m_iLen;
	// 		// 		myparsedCANmsg.STATUS		= myCANmsg.STATUS;
	// 		// 		myparsedCANmsg.DATA         = stringUtils::to_string_byte( myCANmsg.DATA, myCANmsg.m_iLen );
	// 		// 		myparsedCANmsg.message_name = stringUtils::to_string(myCANmsg.m_iID);
	// 		// 	}
	// 		// 	else
	// 		// 	{
	// 		// 		myparsedCANmsg.ID			= -999;
	// 		// 		myparsedCANmsg.LEN			= 0;
	// 		// 		myparsedCANmsg.STATUS		= -999;
	// 		// 		myparsedCANmsg.DATA         = "IGNORE";
	// 		// 		myparsedCANmsg.message_name = "DON'T POST";
	// 		// 	}

	// 	}
	// }

	return myparsedCANmsg;	
}

#endif