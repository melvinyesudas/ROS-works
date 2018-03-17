#!/bin/bash
cd $HOME
cd ~/catkin_ws
source devel/setup.bash
cd src
git clone https://github.com/scanse/sweep-ros.git
cd sweep-ros 
rosdep install -a -y -r
cd ~/catkin_ws
catkin_make
