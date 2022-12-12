# rtsp_tools
Package to interact with Real Time Streaming Protocol (RTSP)

## Installation

* Install robotnik dependencies:
    * robotnik_msgs
    * rcomponent

## RTSP to ROS

Publishes RTSP to ROS topic

### Parameters
* **rtsp_resource**: URL of the RTSP.
* **camera_name**:
* **camera_link**: Name of the camera frame

### Publishers

* **~image_raw(sensors_msgs/Image)**: Publishes camera streaming
* **~camera_info(sensors_msgs/CameraInfo)**: Publishes camera info

### Launch

```bash
roslaunch rtsp_tools rtsp_to_ros.launch
```