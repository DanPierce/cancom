/*
 * cancom_node.cpp
 *
 *  Created on: Nov, 2013
 *  Authors: Dan Pierce
 */

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "std_msgs/String.h"

#include <string>

#include "cancom.h"

class CancomNode{
public:
  CancomNode() : nh_("~"){
    

    // Initialize CAN device
    //can_ = new Cancom();
    
    //can_.init();
  }
  ~CancomNode() {
    this->disconnect();
  }

  
  void init() {
    if (!this->getParameters())
      return;

    bShutDown = can_.init(sensor_type,port_);
    
    switch(sensor_type)
    {
      case 0:
        this->pub_ = nh_.advertise<can_msgs::RadarData>(topic_name_, 10); 
        break; 
      case 1:
        this->pub_ = nh_.advertise<can_msgs::TruckData>("/truck_can", 10);
        break;  
    }

    ros::Time::init();

    this->run();
    return;
  }

  

protected:

  void disconnect() {
    nh_.shutdown();
    //delete can_;
    printf("\nCAN Communication Node has been shut down. Bye!\n\n");
  }

  bool getParameters() {
    name_ = ros::this_node::getName();

    nh_.param("publish_rate", pub_rate, .05);
    ROS_INFO_STREAM(name_ << ": Publish Rate: " << pub_rate);
    //if(!nh_.getParam("port", port_)) port_ = std::string("/dev/pcan32");
    // nh_.param("port", port_, std::string("/dev/pcan32"));
    nh_.param("port", port_, std::string("/dev/pcan32"));
    ROS_INFO_STREAM(name_ << ": Port: " << port_);

    nh_.param("topic_name", topic_name_, std::string("/radar"));
    ROS_INFO_STREAM(name_ << ": Topic Name: " << topic_name_);

    nh_.param("sensor_type", sensor_type, 0);
      // 0 - Delphi Radar
      // 1 - Truck Messages
    switch(sensor_type)
    {
      case 0:
        ROS_INFO_STREAM(name_ << ": Delphi Radar");
        break;
      case 1:
        ROS_INFO_STREAM(name_ << ": Truck CAN");
        break;
    }

    return true;

  }

  void run(){
    ros::Timer timer = nh_.createTimer(ros::Duration(pub_rate), &CancomNode::timerCallback,this);


    // Start ROS time
    //tstart = ros::Time::now();

    usleep(3000);

    ros::spin();

    return;
  }
  
  void timerCallback(const ros::TimerEvent& event)
  {
    TPCANRdMsg read_msg;

    // Calculate loop time;
    //tnow = ros::Time::now();
    //dt = 1e-9*(tnow - tstart).nsec;
    //tstart = tnow;

    switch(sensor_type)
    {
      case 0:
        pub_.publish(can_.radarCanMsgCallback());
        break;
      case 1:
        pub_.publish(can_.truckCanMsgCallback());
        break;
    }
    

    // if(bShutDown)
    // {
    //   ROS_ERROR("\n\nCANCOM Node is Shutting Down! (Emergency Stop)\n");
    //   ros::shutdown();
    // }

  } // end timerCallback

  ros::NodeHandle nh_;
  std::string name_;
  // ROS Messages
  ros::Publisher pub_;

  Cancom can_;

  // topics 


  std::string port_;
  std::string topic_name_;
  int sensor_type;
  double pub_rate;




  // ROS Time
  ros::Time tstart;
  ros::Time tnow;
  double secs;
  double dt;
  bool bShutDown;
};//end class CancomNode

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cancom_node");
  CancomNode node;

  node.init();

  return 0;
  
}


