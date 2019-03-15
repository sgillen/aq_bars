# Aqbar
This is a ROS package that uses a Novint Falcon to teleoperate a modified LynxMotion arm. The arm is equiped with tactile bump switches which are rendered as walls on the Falcon. There is also a simulated version of the arm available in this package. 

This package has been tested on Ubuntu 16.04 using ROS-kinetic

The word Aqbar refers to a mythical creature which is half snow leopard, half falcon. 

## Setup and install

You will first need to install libNiFalcon, following the directions here: https://github.com/libnifalcon/libnifalcon
In addition to the directions on their repo I had to add the following to my bashrc:

```shell
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/
```

This is a ROS package. To use it you will need to install ROS-kinetic (http://wiki.ros.org/kinetic). You will then need a catkin workspace. If you already have one you can skip this step other wise run:

```shell
mkdir aqbar_ws/src
cd aqbar_ws
catkin_make
```

THEN clone this repository into aqbar_ws/src

```shell
cd aqbar_ws/src 
git clone https://github.com/sgillen/aqbar
```

You then need to copy the udev rules

```shell
sudo cp udev_rules/99-udev-novint.rules /etc/udev/rules.d*
```


If you want to use the simualted arm, you will also need to clone this package: https://github.com/ros-planning/panda_moveit_config into your workspace, and checkout and kinetic-devel branch

```shell
cd aqbar_ws/src 
git clone https://github.com/ros-planning/panda_moveit_config/
cd panda_moveit_config
git checkout origin/kinetic-devel 
```

Finally we can build the two packages

```shell
cd aqbar_ws
catkin_make
```

## Running launch files

You should now be able the simulated code with

```shell
roslaunch aqbar sim.launch
```

Or the physical code with

```shell
roslaunch aqbar ard.launch
``

