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
- [tello_driver](https://github.com/michaelwachl/autonomous_drone_plant_detection/tree/master/tello_driver)
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


Useful Websites
----------------


Using Virtual Environment with ROS
----------------------------------

You can used ROS also in a virtual environment.

1. Install Miniconda with Python 2.7 and create your environment

[https://docs.conda.io/en/latest/miniconda.html](url)


2. Add your miniconda export PATH and activation to your .bashrc file
```
export PATH="/home/tello18/miniconda2/bin:$PATH"
```
```
source activate YOUR_ENV
```


3. Source the packages
```
source devel/setup.bash
```

The source command needs to be done everytime you open a new terminal, because it's outside of .bashrc.
If you add it to your .bashrc file the virtual environment won't work. Let me know if there is a other solution.
You may need to install additional packages like catkin_pkg with `pip install rospkg catkin_pkg`
For more information read:  
[https://github.com/ros-infrastructure/rep/blob/master/rep-0008.rst](url)  
[http://docs.ros.org/jade/api/catkin/html/howto/format2/installing_python.html](url)  
[https://medium.com/codeda/ros-run-python-in-virtual-env-8c579304b9c9](url)  
[https://answers.ros.org/question/285483/running-ros-in-a-tensorflow-virtual-environment/](url)


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



