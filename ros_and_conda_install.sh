#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   	echo "This script must be run as root" 
   	exit 1
else
	#Update and Upgrade
	echo "Updating and Upgrading"
	apt-get update && sudo apt-get upgrade -y

	echo "Install mini conda"
	wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
	chmod +x Miniconda3-latest-Linux-x86_64.sh
	./Miniconda3-latest-Linux-x86_64.sh

	
	echo "Create new environment"
	conda create -n ros_env python=2.7

	# install ROS melodic
	echo "install ROS melodic"
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
	sudo apt update
	sudo apt install ros-melodic-desktop-full
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
	sudo rosdep init
	rosdep update

	echo "Create ROS workspace"
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
	source devel/setup.bash
