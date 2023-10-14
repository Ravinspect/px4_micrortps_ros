cmake_minimum_required(VERSION 3.0.2)
project(px4_micrortps_ros)

## Compile as C++11, supported in ROS Kinetic and newer


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  std_msgs
  px4_msgs
  eigen_conversions
  sensor_msgs
  geometry_msgs
  mavros
  mavros_msgs
)

# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
find_package(Eigen3)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


find_package(fastrtps REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES px4_micrortps_ros
  EIGEN3
  eigen_conversions
)

set(RTPS_GENERATED_LIB_SOURCES
src/core/battery_status/battery_status.cxx
src/core/battery_status/battery_statusPublisher.cxx
src/core/battery_status/battery_statusSubscriber.cxx
src/core/battery_status/battery_statusPubSubTypes.cxx
src/core/collision_constraints/collision_constraints.cxx
src/core/collision_constraints/collision_constraintsPublisher.cxx
src/core/collision_constraints/collision_constraintsSubscriber.cxx
src/core/collision_constraints/collision_constraintsPubSubTypes.cxx
src/core/debug_array/debug_array.cxx
src/core/debug_array/debug_arrayPublisher.cxx
src/core/debug_array/debug_arraySubscriber.cxx
src/core/debug_array/debug_arrayPubSubTypes.cxx
src/core/debug_key_value/debug_key_value.cxx
src/core/debug_key_value/debug_key_valuePublisher.cxx
src/core/debug_key_value/debug_key_valueSubscriber.cxx
src/core/debug_key_value/debug_key_valuePubSubTypes.cxx
src/core/debug_value/debug_value.cxx
src/core/debug_value/debug_valuePublisher.cxx
src/core/debug_value/debug_valueSubscriber.cxx
src/core/debug_value/debug_valuePubSubTypes.cxx
src/core/debug_vect/debug_vect.cxx
src/core/debug_vect/debug_vectPublisher.cxx
src/core/debug_vect/debug_vectSubscriber.cxx
src/core/debug_vect/debug_vectPubSubTypes.cxx
src/core/input_rc/input_rc.cxx
src/core/input_rc/input_rcPublisher.cxx
src/core/input_rc/input_rcSubscriber.cxx
src/core/input_rc/input_rcPubSubTypes.cxx
src/core/mavlink_log/mavlink_log.cxx
src/core/mavlink_log/mavlink_logPublisher.cxx
src/core/mavlink_log/mavlink_logPubSubTypes.cxx
src/core/mavlink_log/mavlink_logSubscriber.cxx
src/core/offboard_control_mode/offboard_control_mode.cxx
src/core/offboard_control_mode/offboard_control_modePublisher.cxx
src/core/offboard_control_mode/offboard_control_modeSubscriber.cxx
src/core/offboard_control_mode/offboard_control_modePubSubTypes.cxx
src/core/onboard_computer_status/onboard_computer_status.cxx
src/core/onboard_computer_status/onboard_computer_statusPublisher.cxx
src/core/onboard_computer_status/onboard_computer_statusSubscriber.cxx
src/core/onboard_computer_status/onboard_computer_statusPubSubTypes.cxx
src/core/position_setpoint/position_setpoint.cxx
src/core/position_setpoint/position_setpointPublisher.cxx
src/core/position_setpoint/position_setpointSubscriber.cxx
src/core/position_setpoint/position_setpointPubSubTypes.cxx
src/core/position_setpoint_triplet/position_setpoint_triplet.cxx
src/core/position_setpoint_triplet/position_setpoint_tripletPublisher.cxx
src/core/position_setpoint_triplet/position_setpoint_tripletSubscriber.cxx
src/core/position_setpoint_triplet/position_setpoint_tripletPubSubTypes.cxx
src/core/satellite_info/satellite_info.cxx
src/core/satellite_info/satellite_infoPublisher.cxx
src/core/satellite_info/satellite_infoSubscriber.cxx
src/core/satellite_info/satellite_infoPubSubTypes.cxx
src/core/sensor_combined/sensor_combined.cxx
src/core/sensor_combined/sensor_combinedPublisher.cxx
src/core/sensor_combined/sensor_combinedSubscriber.cxx
src/core/sensor_combined/sensor_combinedPubSubTypes.cxx
src/core/telemetry_status/telemetry_status.cxx
src/core/telemetry_status/telemetry_statusPublisher.cxx
src/core/telemetry_status/telemetry_statusSubscriber.cxx
src/core/telemetry_status/telemetry_statusPubSubTypes.cxx
src/core/timesync/timesync.cxx
src/core/timesync/timesyncPublisher.cxx
src/core/timesync/timesyncSubscriber.cxx
src/core/timesync/timesyncPubSubTypes.cxx
src/core/vehicle_trajectory_waypoint_desired/vehicle_trajectory_waypoint_desired.cxx
src/core/vehicle_trajectory_waypoint_desired/vehicle_trajectory_waypoint_desiredPublisher.cxx
src/core/vehicle_trajectory_waypoint_desired/vehicle_trajectory_waypoint_desiredSubscriber.cxx
src/core/vehicle_trajectory_waypoint_desired/vehicle_trajectory_waypoint_desiredPubSubTypes.cxx
src/core/vehicle_visual_odometry/vehicle_visual_odometry.cxx
src/core/vehicle_visual_odometry/vehicle_visual_odometryPublisher.cxx
src/core/vehicle_visual_odometry/vehicle_visual_odometrySubscriber.cxx
src/core/vehicle_visual_odometry/vehicle_visual_odometryPubSubTypes.cxx
src/core/vehicle_trajectory_waypoint/vehicle_trajectory_waypoint.cxx
src/core/vehicle_trajectory_waypoint/vehicle_trajectory_waypointPublisher.cxx
src/core/vehicle_trajectory_waypoint/vehicle_trajectory_waypointSubscriber.cxx
src/core/vehicle_trajectory_waypoint/vehicle_trajectory_waypointPubSubTypes.cxx
src/core/vehicle_trajectory_bezier/vehicle_trajectory_bezier.cxx
src/core/vehicle_trajectory_bezier/vehicle_trajectory_bezierPublisher.cxx
src/core/vehicle_trajectory_bezier/vehicle_trajectory_bezierSubscriber.cxx
src/core/vehicle_trajectory_bezier/vehicle_trajectory_bezierPubSubTypes.cxx
src/core/vehicle_status/vehicle_status.cxx
src/core/vehicle_status/vehicle_statusPublisher.cxx
src/core/vehicle_status/vehicle_statusSubscriber.cxx
src/core/vehicle_status/vehicle_statusPubSubTypes.cxx
src/core/vehicle_odometry/vehicle_odometry.cxx
src/core/vehicle_odometry/vehicle_odometryPublisher.cxx
src/core/vehicle_odometry/vehicle_odometrySubscriber.cxx
src/core/vehicle_odometry/vehicle_odometryPubSubTypes.cxx
src/core/vehicle_mocap_odometry/vehicle_mocap_odometry.cxx
src/core/vehicle_mocap_odometry/vehicle_mocap_odometryPublisher.cxx
src/core/vehicle_mocap_odometry/vehicle_mocap_odometrySubscriber.cxx
src/core/vehicle_mocap_odometry/vehicle_mocap_odometryPubSubTypes.cxx
src/core/vehicle_local_position/vehicle_local_position.cxx
src/core/vehicle_local_position/vehicle_local_positionPublisher.cxx
src/core/vehicle_local_position/vehicle_local_positionSubscriber.cxx
src/core/vehicle_local_position/vehicle_local_positionPubSubTypes.cxx
src/core/vehicle_local_position_setpoint/vehicle_local_position_setpoint.cxx
src/core/vehicle_local_position_setpoint/vehicle_local_position_setpointPublisher.cxx
src/core/vehicle_local_position_setpoint/vehicle_local_position_setpointSubscriber.cxx
src/core/vehicle_local_position_setpoint/vehicle_local_position_setpointPubSubTypes.cxx
src/core/vehicle_control_mode/vehicle_control_mode.cxx
src/core/vehicle_control_mode/vehicle_control_modePublisher.cxx
src/core/vehicle_control_mode/vehicle_control_modeSubscriber.cxx
src/core/vehicle_control_mode/vehicle_control_modePubSubTypes.cxx
src/core/vehicle_command/vehicle_command.cxx
src/core/vehicle_command/vehicle_commandPublisher.cxx
src/core/vehicle_command/vehicle_commandSubscriber.cxx
src/core/vehicle_command/vehicle_commandPubSubTypes.cxx
src/core/vehicle_attitude/vehicle_attitude.cxx
src/core/vehicle_attitude/vehicle_attitudePublisher.cxx
src/core/vehicle_attitude/vehicle_attitudeSubscriber.cxx
src/core/vehicle_attitude/vehicle_attitudePubSubTypes.cxx
src/core/vehicle_angular_velocity/vehicle_angular_velocity.cxx
src/core/vehicle_angular_velocity/vehicle_angular_velocityPublisher.cxx
src/core/vehicle_angular_velocity/vehicle_angular_velocitySubscriber.cxx
src/core/vehicle_angular_velocity/vehicle_angular_velocityPubSubTypes.cxx
src/core/trajectory_waypoint/trajectory_waypoint.cxx
src/core/trajectory_waypoint/trajectory_waypointPublisher.cxx
src/core/trajectory_waypoint/trajectory_waypointSubscriber.cxx
src/core/trajectory_waypoint/trajectory_waypointPubSubTypes.cxx
src/core/trajectory_setpoint/trajectory_setpoint.cxx
src/core/trajectory_setpoint/trajectory_setpointPublisher.cxx
src/core/trajectory_setpoint/trajectory_setpointSubscriber.cxx
src/core/trajectory_setpoint/trajectory_setpointPubSubTypes.cxx
src/core/trajectory_bezier/trajectory_bezier.cxx
src/core/trajectory_bezier/trajectory_bezierPublisher.cxx
src/core/trajectory_bezier/trajectory_bezierSubscriber.cxx
src/core/trajectory_bezier/trajectory_bezierPubSubTypes.cxx
src/core/distance_sensor/distance_sensor.cxx
src/core/distance_sensor/distance_sensorPublisher.cxx
src/core/distance_sensor/distance_sensorPubSubTypes.cxx
src/core/distance_sensor/distance_sensorSubscriber.cxx
src/core/vehicle_land_detected/vehicle_land_detected.cxx
src/core/vehicle_land_detected/vehicle_land_detectedPublisher.cxx
src/core/vehicle_land_detected/vehicle_land_detectedPubSubTypes.cxx
src/core/vehicle_land_detected/vehicle_land_detectedSubscriber.cxx
)


## Declare a cpp library
 add_library(px4_micrortps_ros ${RTPS_GENERATED_LIB_SOURCES} 
    src/nodelet/battery_status_publisher_nodelet.cpp
    src/nodelet/collision_constraints_publisher_nodelet.cpp
    src/nodelet/debug_array_subscriber_nodelet.cpp
    src/nodelet/debug_key_value_subscriber_nodelet.cpp
    src/nodelet/debug_value_subscriber_nodelet.cpp
    src/nodelet/debug_vect_subscriber_nodelet.cpp
    src/nodelet/input_rc_publisher_nodelet.cpp
    src/nodelet/mavlink_log_publisher_nodelet.cpp
    src/nodelet/offboard_control_mode_publisher_nodelet.cpp
    src/nodelet/onboard_computer_status_subscriber_nodelet.cpp
    src/nodelet/position_setpoint_subscriber_nodelet.cpp
    src/nodelet/position_setpoint_triplet_subscriber_nodelet.cpp
    src/nodelet/satellite_info_publisher_nodelet.cpp
    src/nodelet/sensor_combined_publisher_nodelet.cpp
    src/nodelet/telemetry_status_subscriber_nodelet.cpp
    src/nodelet/vehicle_trajectory_waypoint_desired_publisher_nodelet.cpp
    src/nodelet/vehicle_visual_odometry_subscriber_nodelet.cpp
    src/nodelet/vehicle_trajectory_waypoint_subscriber_nodelet.cpp
    src/nodelet/vehicle_trajectory_bezier_subscriber_nodelet.cpp
    src/nodelet/vehicle_status_publisher_nodelet.cpp
    src/nodelet/vehicle_odometry_publisher_nodelet.cpp
    src/nodelet/vehicle_mocap_odometry_subscriber_nodelet.cpp
    src/nodelet/vehicle_local_position_publisher_nodelet.cpp
    src/nodelet/vehicle_local_position_setpoint_subscriber_nodelet.cpp
    src/nodelet/vehicle_control_mode_publisher_nodelet.cpp
    src/nodelet/vehicle_command_subscriber_nodelet.cpp
    src/nodelet/vehicle_attitude_publisher_nodelet.cpp
    src/nodelet/vehicle_angular_velocity_publisher_nodelet.cpp
    src/nodelet/trajectory_waypoint_publisher_nodelet.cpp
    src/nodelet/trajectory_setpoint_subscriber_nodelet.cpp
    src/nodelet/trajectory_bezier_subscriber_nodelet.cpp
    src/nodelet/timesync_publisher_nodelet.cpp
    src/nodelet/timesync_subscriber_nodelet.cpp
    src/nodelet/distance_sensor_nodelet.cpp
    src/nodelet/hello_world.cpp src/nodelet/hello_world1.cpp
    src/nodelet/vehicle_land_detected_publisher_nodelet.cpp
    src/extras/dummy_position_generator_nodelet.cpp
    src/extras/vehicle_visual_odometry_manager_nodelet.cpp
    src/extras/vehicle_local_position_manager_nodelet.cpp
 )

target_link_libraries(px4_micrortps_ros
  fastrtps
  ${catkin_LIBRARIES}
)

install(TARGETS px4_micrortps_ros
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# install the include directory
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS
  launch/prod/micrortps_agent_build_prod.sh
  launch/prod/micrortps_agent_runner_prod.sh
  launch/test/micrortps_agent_build_test.sh
  launch/test/micrortps_agent_runner_test.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
