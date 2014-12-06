# ERRbot
======

Repository for the Final Project in Computational Robotics, a class at Olin College Fall 2014.

This repo contains code for a robot which searches and locates known objects in an enclosed space. 

## To Run

```
roslaunch neato_node bringup.launch host:=IP_ADDRESS_OF_THE_PI
rosrun ERRbot ERRbot.launch
roslaunch neato_2dnav hector_mapping_neato.launch
(later) roslaunch hector_slam hector_mappping.launch 

```

## Neccessary Installs 

```
Install hector_slam and hector_worldmodel

sudo apt-get install ros-hydro-hector-slam
sudo apt-get install ros-hydro-hector-worldmodel

```