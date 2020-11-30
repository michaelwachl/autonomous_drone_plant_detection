# Environmet Sensor Extension for Tello Drone 


![GUI Info](../doc/GUI_info.png)
![GUI Video](../doc/GUI_video.png)


## Overview

This is a [ROS] package used to to convey drone and PC status and video information and visualize them. The absolute relative position coordinates of the odometry and the SLAM estimate are shown in the position data section.  Additionally,  the remaining flight time, height, speed, flight mode and battery state of the drone are shown in  the  flight data area. The Wi-Fi strength of the PC and Wi-FI quality, as  well  as battery percentage of the drone and the system utility of the base station PC, are displayed as bars. This includes CPU and memory utilization and temperature of the computer. Interactive buttons can be used to start land and fly missions with the drone and connect/disconnect.The mission section is the foundation for autonomous tasks. In another tab, the video stream and sequences can be displayed and recorded when needed and pictures can be taken with a higher resolution. The GUI interacts with the drone over the Tello driver packages and can be easily expanded for future requirements.

**Keywords:** RQT
### License

The source code is released under a [MIT license](../LICENSE).

**Author: Michael Wachl**
Affiliation: 
* [Siemens](https://new.siemens.com/global/en.html)<br />
* [TUM-RCS](https://www.ei.tum.de/rcs/startseite/)<br />

Maintainer: Michael Wachl, michael.wachl@tum.de

The package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.



## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- Qt (graphics library)
- Numpy


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	catkin_make


## Usage
If no roscore is running, start one with 
    
    roscore

Then Run the RQT GUI node with

	rqt --standalone tello_rqt_gui --force-discover

or 

```
rqt
``` 

and look for the flybot plugin in the list.


#### Subscribed Topics

* **`/tello/picture_update`** String
* **`/tello/status`** TelloStatus
* **`/tello/connection_state`** String
* **`/tello/camera/image_raw`** Image
* **`/tello/odom`** Odometry
* **`/orb_slam2_mono/pose`** PoseStamped
* **`/tello_controller/slam_real_world_scale`** Float32
* **`/tello_controller/mission_state`** String

#### Published Topics

* **`/tello/connect`**  Empty
* **`/tello/disconnect`** Empty
* **`/tello/take_picture`** Empty
* **`/tello_controller/mission_command`** String


## Bugs & Feature Requests

To change UI, edit the following file:  
[TelloPlugin.ui](src/tello_rqt/resource/TelloPlugin.ui)

afterwards run 
```
GUI_ui_to_py.sh
```
to generate the according Python file.

If PC utility indicator doesn't work, change the internal temperature name in the [system_utility.py](src/tello_rqt/system_utility.py) script.


Please report bugs and request features using the [Issue Tracker](../../issues).

