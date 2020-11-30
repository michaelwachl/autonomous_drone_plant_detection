Flybot: Autonomous Plant Detection and Localization on a Micro-Drone
============================

![alt text](doc/tello_with_sensor.png)


This Project is about autonomous navigation of an micro aerial vehicle (MAV) and part of my master thesis.
The micro drone [Tello EDU](https://www.ryzerobotics.com/de/tello-edu) is used and controlled in ROS with a remote PC.

**TO DO LIST**
- [x] Tello driver and messages
- [x] RQT GUI implementation
- [x] Uncompressed video topic
- [x] Keybord control
- [x] 3D positions as odom
- [x] 3D positions and point cloud map with orbslam 2
- [x] PID position controller and mission commander
- [x] Object dection with costum data set and YOLOv4
- [x] Object tracking with costum data set and YOLOv4
- [x] Scale routine
- [ ] Object localization with YOLOv4 bounding box
- [ ] Object localization with pointclould and bounding box
- [ ] Plausability check and other tests
  - [ ] Position drift hover, circle path with odom and slam position
  - [ ] Bounding box 3D lines
  - [ ] Size of training set
  - [ ] Miss or no detections of a video sequenz
  - [ ] Induce erros

Table of contents
=================

<!--ts-->
   * [Table of contents](#table-of-contents)
   * [Introduction](#introduction)
   * [Getting Started](#getting-started)
      * [Prerequisites and Installing](#prerequisites-and-installing)
      * [Project Wiki](#wiki)
      * [Useful Websites](#useful-websites)
   * [Flybot GUI](#flybot-gui)
   * [Authors](#authors)
   * [Similar Projects](#similar-projects)
   * [License](#license)
   * [Notes](#notes)
<!--te-->

Introduction
============


Getting Started
===============
This run this project several packages need to be downloaded and installed.
```
git clone --recursive mygit
```

To compile them run:
```
caktin_make
```
in from your workspace folder in your terminal.  

The minimum required packages are:
- [tello_driver](tello_driver) Used for communication with drone (forked and extended)
- [tello_description](tello_description)  Used for visualization and links of drone (forked and extended)
- [tello_controller](/tello_controller) Used to do missions and position control
- [tello_rqt_gui](tello_rqt_gui)  Used to send commands, shoot videos and fotos display states
- [darknet_ros_yolov4](darknet_ros_yolov4) Used to do detection of tomato plants and output bounding boxes (forked and extended)
- [sort_track](sort_track) Used to track objects/bounding boxes (forked and extended)
- [orb_slam_2_ros](orb_slam_2_ros) Used to map environment, output point cloud and pose estimation (forked and extended)
- [localize_plants](localize_plants)  Used to calculate plant location with bonding box and/or point cloud
- [pointcloud_editor](pointcloud_editor) Used to process point cloud


Some other useful packages are:
- [environment_sensor](environment_sensor) Used to connect with sensor and publish sensor data
- [environment_sensor_plot](environment_sensor_plot) Used to create plots of sensor data

Visit each package for further instructions and explanations. 

Prerequisites and Installing
----------------------------

#### Tested on System
* Linux Ubuntu 18.04.4 LTS
* ROS Melodic
* Python 2.7
* [python_qt_binding](https://github.com/ros-visualization/python_qt_binding) (Qt 5)

#### Python Packages
If you used miniconda for your environment you don't need to pip install libaries like numpy, qt, pyqt.
All packages should be covered by creation. 


Wiki
-----
For instruction on how to set up your machine, CUDA, Darknet and other useful tip visit the [Wiki](https://github.com/michaelwachl/autonomous_drone_plant_detection/wiki) of this project. 


Useful Websites
----------------
#### Tello
* [ROS Tello Driver](http://wiki.ros.org/tello_driver)
* [tello_driver](https://github.com/anqixu/tello_driver)
* [tello_driver](https://github.com/appie-17/tello_driver)
* [TelloPy](https://github.com/hanyazou/TelloPy)

#### SLAM

#### Object Detection

#### Object Tracking



Flybot GUI
==========


Authors
=======

* **Author: Michael Wachl**
* Affiliation: 
  * [Siemens](https://new.siemens.com/global/en.html)<br />
  * [TUM-RCS](https://www.ei.tum.de/rcs/startseite/)<br />

Maintainer: Michael Wachl, michael.wachl@tum.de


License
=======

This project is licensed under the MIT License - see the [LICENSE](LICENSE.md) file for details

Acknowledgments
===============

* Hat tip to anyone whose code was used
* Inspiration
* etc

Similar Projects
================


Notes
=====



In Empy may be missing, install it
========================================================================
pip install empy

