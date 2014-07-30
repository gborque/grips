grips
=====

<img align="right" src="./grips_description/resources/bimanual_gazebo.png" width="350" />

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC).

This instructions are for the **grips_robotiq** branch. This branch is under constant change and may become **unstable**.

**Maintainer:** Francisco Suárez Ruiz, [http://www.romin.upm.es/fsuarez/](http://www.romin.upm.es/fsuarez/)

### Documentation

  * See the installation instructions below.
  * This repository.
  * Throughout the various files in the packages.
  * For questions, please use [http://answers.ros.org](http://answers.ros.org)

### Build Status

[![Build Status](https://travis-ci.org/fsuarez6/grips.svg?branch=grips-robotiq)](https://travis-ci.org/fsuarez6/grips)


## Installation

### Basic Requirements

  1. Install [ROS Hydro](http://wiki.ros.org/hydro/Installation/Ubuntu) (**Desktop Install** Recommended)
  2. Install [Gazebo 1.9](http://gazebosim.org/wiki/1.9/install)
  
```
sudo apt-get install ros-hydro-desktop gazebo
``` 

### Repository Installation

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository
```
wstool init .
wstool merge https://raw.github.com/gborque/grips/grips-robotiq/grips.rosinstall
wstool update
``` 
Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro hydro -y
``` 
Now compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source /opt/ros/hydro/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try any of the `.launch` files in the `grips_gazebo` package: (e.g. `grips_robotiq.launch`)
```
roslaunch grips_gazebo grips_robotiq.launch
``` 

## Changelog
### 0.2.0 (2013-11-11)
* Include bimanual setup

### 0.1.0 (2013-10-28)
* Initial Release

## Roadmap

### 0.3.0 (2013-11-11)
* Replace grips collision meshes by simplified primitives (boxes, cylinders, etc.)
* Add collision avoidance capabilities during teleoperation

## Tutorials
TODO
