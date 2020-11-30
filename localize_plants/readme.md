# Localize Plants with tracked bounding boxes

## Overview
![architecture](../doc/localization_arch.png)
![localize](../doc/localization.png)




**Keywords:** YOLO, Object Detection, Tracking and Localization

### License

The source code is released under a [MIT license](../LICENSE).

**Author: Michael Wachl**  
Affiliation: 
* [Siemens](https://new.siemens.com/global/en.html)<br />
* [TUM-RCS](https://www.ei.tum.de/rcs/startseite/)<br />

Maintainer: Michael Wachl, michael.wachl@tum.de

This package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


### Publications

If you use this work in an academic context, please cite the following publication(s):

* Michael Wachl: **Monocular Vision-Based Indoor Object Detection and Localization on Autonomous MAV**. 

        @inproceedings{Wachl2020,
            author = {Michael Wachl},
            title = {{Monocular Vision-Based Indoor Object Detection and Localization on Autonomous MAV}},
            year = {2020}
        }


## Installation

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [darknet_ros package](/../darknet_ros)
- [sort_track package](/../sort_track)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	catkin_make


## Usage/Launch



## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

* **...**


## Nodes



#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])


#### Published Topics

* **`/environment_sensor/scd30`** (environment_sensor.msg/SCD30)


#### Parameters


## Bugs & Feature Requests

Please report bugs and request features using the Issue Tracker
