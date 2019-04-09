/**
MIT License

Copyright (c) 2019 Michail Kalaitzakis (Unmanned Systems and Robotics Lab, 
University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef DJI_GIMBAL_H
#define DJI_GIMBAL_H

// DJI SDK includes
#include <dji_sdk/Gimbal.h>

// ROS includes
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>

#include "std_srvs/SetBool.h"

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

class dji_gimbal {
public:
	dji_gimbal(ros::NodeHandle& nh);
	~dji_gimbal(){};

	// Publish commands
	void publishGimbalCmd();

private:
	// Subscribers
	ros::Subscriber gimbalAngleSub;
	ros::Subscriber joySub;
	ros::Subscriber cameraInfoSub;
	ros::Subscriber pointSub;

	// Publishers
	ros::Publisher gimbalSpeedPub;
	ros::Publisher gimbalAnglePub;

	// Services
	ros::ServiceServer facedownServ;
	ros::ServiceServer faceupServ;
	ros::ServiceServer setTrackingServ;

	// Callbacks
	void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
	void cameraInfoCallback(const sensor_msgs::CameraInfo& msg);
	void pointCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	
	// Service Callbacks
	bool facedownCallback(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res);	
	bool faceupCallback(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res);
	bool setTrackingCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

	// Functions
	void initializeParam();
	void resetGimbalAngle();
	void faceDownwards();

	// Data
	double fx, fy;
	double lastX,lastY;
	double velT, Kp,Kd;
	double posX, posY;

	bool trackPoint;
	std::string cameraInfoTopic;
	std::string pointTopic;
	int yawAxis, pitchAxis, rollAxis, resetButton, faceDownButton, toggleButton;
	geometry_msgs::Vector3Stamped gimbalAngle;
	geometry_msgs::Vector3Stamped speedCmd;
};

#endif //DJI_GIMBAL_H
