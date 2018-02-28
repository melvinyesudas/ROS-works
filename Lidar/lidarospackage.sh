#!/bin/bash
cd $HOME
if [ -e "$HOME" ] ; then
  echo "$DEFAULTDIR already exists; no action taken"
  echo "Placing Scanse Sweep package into $DEFAULTDIR" 
else 
  echo "$DEFAULTDIR does not exist; no action taken."
  echo "This script expects an existing initialized Catkin Workspace before installing the Scanse Sweep Package."
  exit 1
fi
cd "$DEFAULTDIR"
echo "Installing prerequisites"
source devel/setup.bash
cd src
git clone https://github.com/scanse/sweep-ros.git
cd sweep-ros
# Install the rosdeps -a = all -y = no questions -r = skip errors 
rosdep install -a -y -r
cd $HOME
catkin_make
