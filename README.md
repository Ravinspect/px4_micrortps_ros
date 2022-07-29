This code repository was created for ROS Melodic and RTPS communication instead of Mavlink.

Tested on our enviroment and hardware.

# Environment

* Ubuntu 18.04
* ROS Melodic
* PX4 V1.12.3
* FastDDS v 2.0.2
* FastRTPS-Gen 1.0.4
* foonathan_memory_vendor
* Java 
* Gradle 6.4.1
  
# Hardware

* Pixhawk mini 4
* Jetson Xavier / Ubuntu Linux
* Telem to USB converter
  

# Setup

 *  Follow the PX4's installition from here: https://docs.px4.io/master/en/middleware/micrortps.html  
  
 

## Getting started

* git clone -b melodic --recurse-submodules -j8 https://gitlab.com/ravinspect/px4_micrortps_ros.git
* cd px4_micrortps_ros
* git submodule update --remote

## Running
1) build the micrortps_agent;
    
* build micrortps_agent test env: roslaunch px4_micrortps_ros micrortps_agent_build_test.launch 

* build micrortps_agent prod env: roslaunch px4_micrortps_ros micrortps_agent_build_prod.launch 

2)  PX4  start.
roslaunch quad_3dnav px4_sim.launch

3) Start the micrortps_Agent and all nodes
roslaunch px4_micrortps_ros px4_sitl_rtps.launch 


