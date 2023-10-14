#!/bin/bash
cd $HOME/catkin_ws_rtps/src/px4_micrortps_ros/micrortps_agent/prod/px4_fmu-v5_rtps/build
./micrortps_agent -t UART -d /dev/ttyUSB0 -b 460800
