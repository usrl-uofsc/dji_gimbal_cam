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
#include "djicam.h"

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
	bool loadCameraInfo();
	int is_mono;

	// DJI camera parameters
	int imageWidth;
	int imageHeight;
	int imageChannels;
	int frameSize;
	int mode;

	// Load new frame
	bool grabFrame();
	cv_bridge::CvImage rosMat;

	// Set up topics and call Publishers
	int nCount;
};


#endif //DJI_CAMERA_H