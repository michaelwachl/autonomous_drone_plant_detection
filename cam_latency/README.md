# Topic Monitoring for Tello Drone 

## Overview

This is a [ROS] package listens to topics of the tello driver to compute the bandwith and rate. A time is displayed to be filmed to see the camera latency. This node can also publish a video stream with center lines.


**Keywords:** video latency, rate, bandwidth, camera center lines


### License

The source code is released under a [MIT license](../LICENSE).

**Author: Michael Wachl**
Affiliation: 
* [Siemens](https://new.siemens.com/global/en.html)<br />
* [TUM-RCS](https://www.ei.tum.de/rcs/startseite/)<br />

Maintainer: Michael Wachl, michael.wachl@tum.de**

The package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- matplotlib

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	catkin_make


## Usage
If no roscore is running, start one with 
    
    roscore

Then Run the sensor node(s) with

	rosrun cam_latency compute_latency.py 


## Nodes

`tello_latency`

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])
* **`/tello/camera/image_raw`**  Image
* **`/tello/image_raw/h264`** CompressedImage


#### Published Topics

* **`/tello/camera/image_raw_center`** Image

## Monitored Topics (Bandwidth, Rate)
* **`/tello/image_raw/h264`** sensor_msgs/CompressedImage
* **`/tello/camera/image_raw`** sensor_msgs/Image
* **`/tello/imu`** sensor_msgs/Imu
* **`/tello/status`** tello_driver/TelloStatus

