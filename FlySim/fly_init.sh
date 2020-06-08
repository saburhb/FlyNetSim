#!/bin/sh

git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive

cd ardupilot
#./Tools/scripts/install-prereqs-ubuntu.sh -y  #prereq script location changed
./Tools/environment_install/install-prereqs-ubuntu.sh -y

~/.profile

sudo pip install dronekit
sudo pip install dronekit-sitl
sudo pip install pyzmq


