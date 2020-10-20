Flybot: Autonomous Plant Detection and Localization on a Micro-Drone
============================

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
- [ ] Scale routine
- [ ] Object localization with YOLOv4 bounding box
- [ ] Object localization with pointclould and bounding box
- [ ] Plausability check and other tests


Table of contents
=================

<!--ts-->
   * [Table of contents](#table-of-contents)
   * [Introduction](#introduction)
   * [Getting Started](#getting-started)
      * [Project Wiki](#wiki)
      * [Useful Websites](#useful-websites)
      * [Using Virtual Environment with ROS](#using-virtual-environment-with-ros)
      * [Prerequisites and Installing](#prerequisites-and-installing)
   * [Flybot GUI](#flybot-gui)
   * [Authors](#authors)
   * [License](#license)
   * [Similar Projects](#similar-projects)
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
The minimum required packages are:
- [tello_driver](/tello_driver)
- [tello_description](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/tello_description)
- [tello_controller](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/tello_controller)
- [tello_rqt_gui](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/tello_rqt_gui)
- [YOLOv4](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/tello_driver)
- [orbslam2](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/tello_driver)
- [localize_plants](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/localize_plants)
- [pointcloud_editor](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/pointcloud_editor)
- [sort_track](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/sort_track)

Visit each package for further instructions and explanations. 

To compile them run:
```
caktin_make
```
in from your workspace folder in your terminal. 


Wiki
-----
For instruction on how to set up your machine, CUDA, Darknet and other useful tip visit the [Wiki](https://github.com/michaelwachl/autonomous_drone_plant_detection/wiki) of this project. 

Useful Websites
----------------


Prerequisites and Installing
----------------------------

#### Tested on System
* Linux Ubuntu 18.04.4 LTS
* ROS Melodic
* Python 2.7
* [python_qt_binding](https://github.com/ros-visualization/python_qt_binding) (Qt 5)


#### ROS Packages
You need to install the following ROS-Packages:
*  [libuvc_camera](https://wiki.ros.org/libuvc_camera)
*  [joy](https://wiki.ros.org/joy)

Open your terminal and install the following packages to your environment:  
The ROS [libuvc_camera](https://wiki.ros.org/libuvc_camera) package for the camera drivers
```
sudo apt-get install ros-melodic-libuvc-camera
```
The ROS [joy](https://wiki.ros.org/joy) package joystick drivers 

#### Python Packages
If you used miniconda for your environment you don't need to pip install libaries like numpy, qt, pyqt.
All packages should be covered by creation. 

In this project the following packages are used:


Flybot GUI
==========


Authors
=======

* **Michael Wachl** - 


License
=======

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

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



