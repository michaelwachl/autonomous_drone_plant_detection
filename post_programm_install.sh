#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   	echo "This script must be run as root" 
   	exit 1
else
	#Update and Upgrade
	echo "Updating and Upgrading"
	apt-get update && sudo apt-get upgrade -y

	echo "Install Spotify"
	sudo snap install spotify

	echo "Install Pycharm"
	sudo snap install pycharm-community --classic

	echo "Install Arduino"
	sudo snap install arduino

	echo "Install VLC"
	sudo snap install vlc

	echo "Install OBS"
	sudo snap install obs-studio

	echo "Install gimp"
	sudo snap install gimp

	echo "Install inkscape"
	sudo snap install inkscape

	echo "Install sublime text"
	sudo snap install sublime-text --classic

