# Arducam_RPi_ROS_node 

This repository holds ROS node to work with Arducam MIPI cameras connected to RPi and running Raspbian OS. This node will publish topics under /camera/image name with maximal resolution black and white mono 8-bit images with raw and compressed messages to support different communication bandwidth networks. 

This node configured to work with external trigger for camera module. This option provided by Arducam library which we wrapped with this ROS node.
The FPS rate determined by other node that generates external trigger signal.

Please note that this node was checked for global shutter camera module UC-599 that has native support for OV9281 sensor module in RPi OS.



We allow at launch time of the node to change exposure and gain levels of the sensor. 

Each message is time tagged by current ROS time after image is retrieved.

In order to make sure proper operation of this node it is critical to install ROS melodic at least up to image_transport package.



## Installation Instructions



### Installing RPi OS:

Instruction and installation images could be found [here](https://www.raspberrypi.org/software/operating-systems/). We worked with kernel version 5.4.72, version 10 of Raspbian.



### Installing ROS melodic:

ROS melodic has no precompiled packaged for Raspbian OS and therefore need to compile manually. In order to compile it please go on this [link](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi). Follow instruction up to section 3.1.



In section 3.1 please use following command instead as it is impossible to install full version of all available packages due to libboost compatibility issues of two different packages, therefore we only install up-to image_transport package.



```

rosinstall_generator image_transport --rosdistro melodic --deps --wet-only > melodic-image_transport-wet.rosinstall 

```

Rest of the installation process for ROS melodic should be followed until section 4.



### Installing Arducam driver:

Please follow instructions found on Arducam git repository found [here](https://github.com/ArduCAM/MIPI_Camera/tree/master/RPI).



If all went correctly you will be able to run Arducams example as posted on their git.



### Installing Arducam ROS node

In your's catkin workspace folder inside src folder clone our repository:

```

git clone https://github.com/tau-adl/Arducam_RPi_ROS_node.git

```

Build node:

```

cd ..

catkin_make

source ./devel/setup.bash

```



## Usage



If all went good, you be able to launch the cpp_stream node. Please note that as we mentioned above, the node using external trigger to capture image and if no external trigger is given up to timeout provided to Arducam function, the node will exit.



Launching example setting some gain and exposure values:

```

roslaunch cpp_stream cam_stream.launch gain:=4 exposure:=800

```



The published topics are:

/camera/image

/camera/image/compressed

/camera/image/compressed/parameter_descriptions

/camera/image/compressed/parameter_updates



When node is initializing it will print out minimal and maximal exposure and gain register values.



You are welcome to expand this node functionality as you wish.

## Running full setup RPi and Host PC

First, configure host PC to be ROS master by setting environment variable as bellow, it will be the roscore instance to which RPi will publish the topics:
Run on host:
```
export ROS_MASTER_URI=http://localhost:11311
roscore
``` 
Afterwards log in to RPi through SSH - either from 2 separate sessions then run:
```
export ROS_IP={fill in IP of RPi}
export ROS_MASTER_URI=http://{fill in IP of HOST pc}:11311
sudo pigpiod - (Or set is startup service)
source ~/{folder_where_mpu9150_installed}/devel/setup.bash
rosrun mpu9150_imu_pi imupi_python.py
```
From another SSH session - run:
```
export ROS_IP={fill in IP of RPi}
export ROS_MASTER_URI=http://{fill in IP of HOST pc}:11311
source ~/{folder_where_arducam_stream_installed}/devel/setup.bash
roslaunch cpp_stream cam_stream.launch gain:={desired_gain} exposure:={desired_exposure}
```
On host PC open another 3 (4 for VINS-MONO) terminals:
In terminal A - run:
```
rosrun image_transport republish compressed in:=/camera/image raw out:=/camera_decompressed/image_raw
```
In terminal B - run:
```
source binning_ws/devel/setup.bash
rosrun binning binning_node
```
In terminal C ??? run for ORB-SLAM3:
```
source ./lab3/lab3_ws/devel/setup.bash
roslaunch orb_slam3_wrapper orb_slam3_mono_inertial.launch
```
or
```
source ./vins_mono_ws/devel/setup.bash
roslaunch vins_estimator arducam.launch
```
For VINS-MONO open another terminal ??? D and run RVIZ configuration:
```
source ./vins_mono_ws/devel/setup.bash
roslaunch vins_estimator vins_rviz.launch
```
