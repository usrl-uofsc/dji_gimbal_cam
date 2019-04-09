# dji_gimbal_cam

## dji_camera_node
Reads the feed from a DJI Camera connected to DJI Manifold and publishes the image and camera information on a ROS topic.

## dji_gimbal_node
Reads gimbal information from the gimbal camera. Provides control through ROS topics and a higher level control for tracking a 3d point in the frame of the camera.

## Tested Cases
Tested on a manifold running ROS Indigo.

Controllers:
* M100 (DJI Matrice M100)
* N3 (HexH2O with N3)

Gimballed Cameras (configurations provided):
* Zenmuse Z3
* Zenmuse X3

## Dependencies
* DJI Onboard SDK (https://github.com/dji-sdk/Onboard-SDK/tree/3.7)
* DJI SDK ROS (https://github.com/dji-sdk/Onboard-SDK-ROS/tree/3.7/dji_sdk)
* DJI Manifold Cam (https://github.com/dji-sdk/Manifold-Cam)

## Install
1. Open a fresh terminal on your manifold
2. Source your ROS installation
3. `cd` into your catkin workspace
4. `git clone https://github.com/usrl-uofsc/dji_gimbal_cam.git src/dji_gimbal_cam`
5. `catkin_make --pkg dji_gimbal_cam`

## Run
1. Open a terminal
2. `sudo -s`
	* Allows the Manifold to control the gimbal and receive images
3. Source your ROS workspace
4. `roslaunch dji_gimbal_cam load_rosparam.launch`
	* This will load the default ROS parameters

## Configure
Use `cfg/default_config.yaml` as a base configuration.

`track_point`: Should the gimbal default to following a point (Bool).
`camera_info_topic`: Topic of the camera_info (Str).
`track_point_topic`: Topic which publishes a point to track in the camera's frame (Str).

`yaw_axis`, `pitch_axis`, `roll_axis`, `reset_angle_btn`, `face_down_btn`, `toggle_track_btn`: Configuration for joystick control (Int).

`Kp`, `Kd`: PD controller parameters (Double).


camera_info_url: URL for the camera configuration file (URL).
camera_name: Name of the camera to be recognized by DJI SDK (Str)
camera_frame_id: Frame of the camera (Str).

is_mono: Is the image grayscale (Bool)
transfer: Should the image be transfered to the Lightbridge controller. Increases bandwidth usage (Bool).

## License

### MIT License

Copyright (c) 2019 **Michail Kalaitzakis** (Unmanned Systems and Robotics Lab, 
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

