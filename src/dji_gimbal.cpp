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

#include "dji_gimbal_cam/dji_gimbal.h"

#include <unistd.h>

#include <iostream>

dji_gimbal::dji_gimbal(ros::NodeHandle& nh)
{
	// Configure the gimbal control parameters
	initializeParam();

	// Setup Subscribers
	gimbalAngleSub = nh.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/gimbal_angle", 10, &dji_gimbal::gimbalAngleCallback, this);
	joySub = nh.subscribe("joy", 10, &dji_gimbal::joyCallback, this);
	cameraInfoSub = nh.subscribe(cameraInfoTopic, 10, &dji_gimbal::cameraInfoCallback, this);
	pointSub = nh.subscribe(pointTopic, 10, &dji_gimbal::pointCallback, this);

	// Setup Publishers
	gimbalSpeedPub = nh.advertise<geometry_msgs::Vector3Stamped>("/dji_sdk/gimbal_speed_cmd", 10);
	gimbalAnglePub = nh.advertise<dji_sdk::Gimbal>("/dji_sdk/gimbal_angle_cmd", 10);

	// Setup Services
	facedownServ = nh.advertiseService("facedown", &dji_gimbal::facedownCallback, this);
	faceupServ = nh.advertiseService("faceup", &dji_gimbal::faceupCallback, this);
	setTrackingServ = nh.advertiseService("setGimbalTracking", &dji_gimbal::setTrackingCallback, this);
}

void dji_gimbal::initializeParam()
{
	// Create private nodeHandle to read the launch file
	ros::NodeHandle nh_local("");

	// Load values from launch file
	nh_local.param("track_point", trackPoint, false);
	nh_local.param("camera_info_topic", cameraInfoTopic, std::string("/dji_sdk/camera_info"));
	nh_local.param("track_point_topic", pointTopic, std::string("track_point"));
	nh_local.param("yaw_axis", yawAxis, 0);
	nh_local.param("pitch_axis", pitchAxis, 4);
	nh_local.param("roll_axis", rollAxis, 3);
	nh_local.param("reset_angle_btn", resetButton, 0);
	nh_local.param("face_down_btn", faceDownButton, 2);
	nh_local.param("toggle_track_btn", toggleButton, 3);
	nh_local.param("Kp", Kp, 0.0);

	// Initialize speed command
	speedCmd.vector.x = 0;
	speedCmd.vector.y = 0;
	speedCmd.vector.z = 0;

	// Initialize flags
	velT = 0.75;
}

void dji_gimbal::publishGimbalCmd()
{
	if (trackPoint)
	{
		speedCmd.vector.x = 0;
		
		// Corrections in x,y
		double cx =  posX * Kp;
		double cy = -posY * Kp;
		
		// Crop if outside range
		cx = cx > velT ? velT : (cx < -velT ? -velT : cx);
		cy = cy > velT ? velT : (cy < -velT ? -velT : cy);

		// Add to vector
		speedCmd.vector.y = cy;
		speedCmd.vector.z = cx;

		gimbalSpeedPub.publish(speedCmd);

		// Reset Commands to zero
		speedCmd.vector.y = 0;
		speedCmd.vector.z = 0;
	}
	else
		gimbalSpeedPub.publish(speedCmd);
}

void dji_gimbal::resetGimbalAngle()
{
	// Prepare the reset command
	dji_sdk::Gimbal angleCmd;
	angleCmd.mode |= 0;
	angleCmd.mode |= 1; // for absolute angle
	angleCmd.ts    = 2;
	angleCmd.roll  = 0;
	angleCmd.pitch = 0;
	angleCmd.yaw   = 0;

	gimbalAnglePub.publish(angleCmd);

	sleep(2);
}

void dji_gimbal::faceDownwards()
{
	// Prepare the angle command
	dji_sdk::Gimbal angleCmd;
	angleCmd.mode |= 0;
	angleCmd.mode |= 1; // for absolute angle
	angleCmd.ts    = 2;
	angleCmd.roll  = 0;
	angleCmd.pitch = DEG2RAD(-90);
	angleCmd.yaw   = 0;

	gimbalAnglePub.publish(angleCmd);
}

// Callbacks
void dji_gimbal::gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
	gimbalAngle = *msg;
}

void dji_gimbal::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	// Toggle track flag
	if (msg->buttons[toggleButton] == 1)
		trackPoint = !trackPoint;
	if (msg->buttons[resetButton] == 1)
		resetGimbalAngle();
	else if (msg->buttons[faceDownButton] == 1)
		faceDownwards();
	else
	{
		// Update speed command
		speedCmd.vector.x = -msg->axes[rollAxis];
		speedCmd.vector.y = -msg->axes[pitchAxis];
		speedCmd.vector.z = -msg->axes[yawAxis];
	}
}

void dji_gimbal::cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
{
	// Get the focal length of the camera
	fx = msg.K[0];
	fy = msg.K[4];
}

void dji_gimbal::pointCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	posX = fx*(msg->x/msg->z);
	posY = fy*(msg->y/msg->z);
}

bool dji_gimbal::facedownCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	faceDownwards();
	res.success = true;
	return true;
}

bool dji_gimbal::faceupCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	resetGimbalAngle();
	res.success = true;
	return true;
}

bool dji_gimbal::setTrackingCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	trackPoint = req.data;
	res.success = true;
	return true;
}

////////////////////////////////////////////////////////////
////////////////////////  Main  ////////////////////////////
////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dji_gimbal_control_node");
	ros::NodeHandle nh;

	dji_gimbal gimbalControl(nh);

	ros::Rate rate(30);

	while(ros::ok())
	{
		ros::spinOnce();
		gimbalControl.publishGimbalCmd();

		rate.sleep();
	}

	return 0;
}
