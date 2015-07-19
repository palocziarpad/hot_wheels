#!/bin/bash
echo "update local"
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
echo "2.3 Setup your sources.list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
echo "2.4 Set up your keys"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
echo "2.5 Installation"
sudo apt-get update
echo "Install indigo"
sudo apt-get install ros-indigo-ros-base
echo "Initialize rosdep"
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
echo "Environment setup"
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc


