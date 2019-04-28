#!/bin/sh

    pid=$( ps -A | grep -w FlyNetSim.py | awk '{print $1}')
    for id in $pid
	do
	kill -9 $id
    done;
    pid=$( ps -A | grep sim_vehicle | awk '{print $1}')
    for id in $pid
	do
	kill -9 $id
    done;
    pid=$( ps -A | grep sitl | awk '{print $1}')
    for id in $pid
	do
	kill -9 $id
    done;
    pid=$( ps -A | grep uav-net | awk '{print $1}')
    for id in $pid
	do
	kill -9 $id
    done;

