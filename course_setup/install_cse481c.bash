#! /bin/bash

# Basic stuff
sudo apt-get update
sudo apt-get install -y build-essential vim emacs git tmux python3-dev curl python-pip cmake libgif-dev openssh-server

# MoveIt
sudo apt-get install -y ros-noetic-moveit-*

# Fetch
sudo apt-get install -y ros-noetic-fetch-*

# Kuri Dependencies
sudo apt-get install -y ros-noetic-gazebo-ros-control

# ROS utils
sudo apt-get install -y python-wstool python-catkin-tools python-rosinstall

# NVM
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.8/install.sh | bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" # This loads nvm
nvm install node
npm install -g polymer-cli bower

# Caddy
curl https://getcaddy.com | bash -s personal http.cors
