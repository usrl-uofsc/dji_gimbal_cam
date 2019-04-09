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


/* ROS node to read camera feed from a DJI camera and publish it as a ROS topic
*  Needs a DJI Manifold and DJI compatible camera
*  Tested on DJI Matrice 100 with Zenmuse Z3
*/

#ifndef DJI_CAMERA_H
#define DJI_CAMERA_H

// ROS includes
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"

// DJI camera header
#include "dji_gimbal_cam/djicam.h"

class dji_camera {
public:
	dji_camera(ros::NodeHandle& nh, image_transport::ImageTransport& imageT);
	~dji_camera();

	// Publisher functions
	bool publishAll();

private:
	// ROS Publishers
	image_transport::Publisher imagePub;
	ros::Publisher cameraInfoPub;

	// Camera info
	sensor_msgs::CameraInfo cam_info;
	std::string camera_frame_id;
	bool loadCameraInfo();
	bool is_mono;

	// DJI camera parameters
	cv_bridge::CvImage rosMat;
	int imageWidth;
	int imageHeight;
	int imageChannels;
	int frameSize;
	int mode;
	int nCount;

	// Frame grabber function
	bool grabFrame();
};

#endif //DJI_CAMERA_H
