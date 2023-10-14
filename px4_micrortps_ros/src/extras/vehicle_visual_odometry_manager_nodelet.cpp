#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <thread>
#include <chrono>

#include <array>

#include <ros/assert.h>

// for Covariance types
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <eigen_conversions/eigen_msg.h>

#include  <tf/transform_broadcaster.h>

#include <px4_msgs/VehicleVisualOdometry.h>
#include <px4_msgs/VehicleAngularVelocity.h>
#include <px4_msgs/Timesync.h>

#include  <tf/transform_broadcaster.h>

#include <mavros/utils.h>
#include <mavros/frame_tf.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>


using namespace mavros;

namespace px4_micrortps_ros
{

  class VehicleVisualOdometryManagerNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;
         ros::Subscriber sub_vehicle_angular_velocity;
         ros::Subscriber sub_timesync;
         ros::Publisher pub;

        VehicleVisualOdometryManagerNodelet()
        {

        }
    
        ~VehicleVisualOdometryManagerNodelet()
        {
        }
    private: 
        ros::Time last_transform_stamp;
        geometry_msgs::TransformStamped transform_stamped;  
        uint64_t timestamp;  
        uint64_t timestamp_sample;
    private:

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/position_sent",5, &VehicleVisualOdometryManagerNodelet::callback, this);
            sub_vehicle_angular_velocity = private_nh.subscribe("/px4_micrortps_ros/vehicle_angular_velocity/out",5, &VehicleVisualOdometryManagerNodelet::callbackVehicleAngularVelocity, this);
            sub_timesync  = private_nh.subscribe("/px4_micrortps_ros/timesync/out",5, &VehicleVisualOdometryManagerNodelet::callbackTimesync, this);

            pub = private_nh.advertise<px4_msgs::VehicleVisualOdometry>("/px4_micrortps_ros/vehicle_visual_odometry/in", 10);
        }

        void callbackVehicleAngularVelocity(const px4_msgs::VehicleAngularVelocityPtr input)
        {
            px4_msgs::VehicleAngularVelocity vehicle_angular_velocity = *input;

            // timestamp = vehicle_angular_velocity.timestamp;
            // timestamp_sample = vehicle_angular_velocity.timestamp_sample;

        }


        void callbackTimesync(const px4_msgs::TimesyncPtr input)
        {
            px4_msgs::Timesync timesync = *input;

            timestamp = timesync.timestamp;
            timestamp_sample = timesync.tc1 / 1000;
        }
        
        
        void callback(const geometry_msgs::TransformStampedPtr input)
        {

            transform_stamped = *input;

            ROS_INFO("Vehicle Visual Odometry Manager: %f, %f, %f", transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z);

            ftf::Covariance6d cov {};	// zero initialized

            Eigen::Affine3d tr;
            tf::transformMsgToEigen(transform_stamped.transform, tr);

            if (last_transform_stamp == transform_stamped.header.stamp) {
                ROS_DEBUG_THROTTLE_NAMED(10, "vision_pose", "Vision: Same transform as last one, dropped.");
                return;
            }

            last_transform_stamp = transform_stamped.header.stamp;

            // auto position = mavros::ftf::transform_frame_enu_ned();

            auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));

            auto quat = ftf::transform_orientation_enu_ned( ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));

            auto cov_ned =  ftf::transform_frame_enu_ned(cov);

            ftf::EigenMapConstCovariance6d cov_map(cov_ned.data());

            auto urt_view = Eigen::Matrix<double, 6, 6>(cov_map.triangularView<Eigen::Upper>());

            px4_msgs::VehicleVisualOdometryPtr msg_ptr(new px4_msgs::VehicleVisualOdometry);

            // msg_ptr->timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
            // msg_ptr->timestamp_sample = (std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count())/1000;

            msg_ptr->timestamp = timestamp;
            msg_ptr->timestamp_sample = timestamp_sample;


            msg_ptr->local_frame = 0;

            msg_ptr->x =  position.x();
            msg_ptr->y =  position.y();
            msg_ptr->z =  position.z();

            msg_ptr->q[0] =  quat.x();
            msg_ptr->q[1] =  quat.y();
            msg_ptr->q[2] =  quat.z();
            msg_ptr->q[3] =  quat.w();

            std::cout << "VehicleVisualOdometryPtr 1: "<< msg_ptr << std::endl;
            
            pub.publish(msg_ptr);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleVisualOdometryManagerNodelet, nodelet::Nodelet)
