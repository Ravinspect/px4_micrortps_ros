#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <thread>
#include <chrono>
#include <cmath>
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
#include <px4_msgs/VehicleLocalPosition.h>
#include <px4_msgs/VehicleAttitude.h>

#include  <tf/transform_broadcaster.h>

#include <mavros/utils.h>
#include <mavros/frame_tf.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <mavros/mavros_plugin.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

using namespace mavros;

namespace px4_micrortps_ros
{

  class VehicleLocalPositionManagerNodelet : public nodelet::Nodelet
  {
    public:


        //  ros::Publisher pub;
        VehicleLocalPositionManagerNodelet()
        {

        }
    
        ~VehicleLocalPositionManagerNodelet()
        {
        }

    private:
        ros::Time last_transform_stamp; 

        ros::Subscriber sub_vehicle_local_position;
        ros::Subscriber sub_vehicle_attitude;
        ros::Subscriber sub_vehicle_angular_velocity;

        ros::Publisher local_position_pub;
        ros::Publisher local_velocity_pub;
        ros::Publisher local_odom_pub;
        ros::Publisher imu_data_pub;

        Eigen::Vector3d gyro_frd;
        Eigen::Vector3d gyro_flu;
        Eigen::Quaterniond ned_aircraft_orientation;
        Eigen::Quaterniond enu_baselink_orientation;

        std::string frame_id = "world";		//!< frame for Pose
        std::string tf_frame_id = "map";	//!< origin for TF
        std::string tf_child_frame_id = "base_link";	//!< frame for TF

        ftf::Covariance3d linear_acceleration_cov;
        ftf::Covariance3d angular_velocity_cov;
        ftf::Covariance3d orientation_cov;
        ftf::Covariance3d unk_orientation_cov;
        ftf::Covariance3d magnetic_cov;
    private:

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the VehicleLocalPositionManagerNodelet");

            local_position_pub = private_nh.advertise<geometry_msgs::PoseStamped>("/px4_micrortps_ros/local_position/pose", 10);
            local_velocity_pub = private_nh.advertise<geometry_msgs::TwistStamped>("/px4_micrortps_ros/local_position/velocity", 10);
            local_odom_pub = private_nh.advertise<nav_msgs::Odometry>("/px4_micrortps_ros/local_position/odom",10);
            imu_data_pub = private_nh.advertise<sensor_msgs::Imu>("/px4_micrortps_ros/imu_data",10);

            sub_vehicle_attitude = private_nh.subscribe("/px4_micrortps_ros/vehicle_attitude/out",5, &VehicleLocalPositionManagerNodelet::callbackVehicleAttitude, this);
            sub_vehicle_angular_velocity = private_nh.subscribe("/px4_micrortps_ros/vehicle_angular_velocity/out",5, &VehicleLocalPositionManagerNodelet::callbackVehicleAngularVelocity, this);
            sub_vehicle_local_position = private_nh.subscribe("/px4_micrortps_ros/vehicle_local_position/out",5, &VehicleLocalPositionManagerNodelet::callbackVehicleLocalPosition, this);

            setup_covariance(linear_acceleration_cov, 0.0003);
            setup_covariance(angular_velocity_cov, 0.02 * (M_PI / 180.0));
            setup_covariance(orientation_cov, 1.0);
            setup_covariance(magnetic_cov, 0.0);
            setup_covariance(unk_orientation_cov, 0.0);
        
        }
        
        void callbackVehicleLocalPosition(const px4_msgs::VehicleLocalPositionPtr msgs)
        {
            //std::cout << "Local Position TransformStampedConstPtr: " << input << std::endl;
            px4_msgs::VehicleLocalPosition vlp = *msgs;

            ROS_INFO("Local Position Manager: %f, %f, %f", vlp.x, vlp.y, vlp.z);

            //--------------- Transform FCU position and Velocity Data ---------------//
            auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(vlp.x, vlp.y, vlp.z));
            auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(vlp.vx, vlp.vy, vlp.vz));

            auto imu_enu_msg = boost::make_shared<sensor_msgs::Imu>();
            auto imu_ned_msg = boost::make_shared<sensor_msgs::Imu>();

            //TODO mavros params px4_config.yaml
            std::string imu_frame_id = "imu_frame";
            // Fill message header
            std_msgs::Header out;
            out.frame_id = imu_frame_id;
            //out.stamp = vlp.timestamp;
            imu_enu_msg->header = out;//TODO timestamp sync need... m_uas->synchronized_header(imu_frame_id, vlp.timestamp);
            //imu_ned_msg->header = m_uas->synchronized_header("aircraft", time_boot_ms);

            // Convert from Eigen::Quaternond to geometry_msgs::Quaternion
            tf::quaternionEigenToMsg(enu_baselink_orientation, imu_enu_msg->orientation);
            //tf::quaternionEigenToMsg(orientation_ned, imu_ned_msg->orientation);

            // Convert from Eigen::Vector3d to geometry_msgs::Vector3
            tf::vectorEigenToMsg(gyro_flu, imu_enu_msg->angular_velocity);
            //tf::vectorEigenToMsg(gyro_frd, imu_ned_msg->angular_velocity);

            // Eigen::Vector3d from HIGHRES_IMU or RAW_IMU, to geometry_msgs::Vector3
            //TODO raw imu data from magnetide (px4_msgs::SensorMag) tf::vectorEigenToMsg(linear_accel_vec_flu, imu_enu_msg->linear_acceleration);
            //tf::vectorEigenToMsg(linear_accel_vec_frd, imu_ned_msg->linear_acceleration);

            // Pass ENU msg covariances
            imu_enu_msg->orientation_covariance = orientation_cov; // TODO "orientation_stdev", orientation_stdev, 1.0 orientation_cov;
            imu_enu_msg->angular_velocity_covariance =  angular_velocity_cov; //TODO  imu_nh.param("angular_velocity_stdev", angular_stdev, 0.02 * (M_PI / 180.0)); angular_velocity_cov;
            imu_enu_msg->linear_acceleration_covariance = linear_acceleration_cov;  //TODO "linear_acceleration_stdev", linear_stdev, 0.0003 linear_acceleration_cov; 

            // Pass NED msg covariances
            // imu_ned_msg->orientation_covariance = orientation_cov;
            // imu_ned_msg->angular_velocity_covariance = angular_velocity_cov;
            // imu_ned_msg->linear_acceleration_covariance = linear_acceleration_cov;


            // /** Store attitude in aircraft NED
            //  *  @snippet src/plugins/imu.cpp store_ned
            //  */
            // // [store_enu]
            // m_uas->update_attitude_imu_ned(imu_ned_msg);
            // // [store_ned]

            // /** Publish only base_link ENU message
            //  *  @snippet src/plugins/imu.cpp pub_enu
            //  */
            // // [pub_enu]
            // imu_pub.publish(imu_enu_msg);

            auto enu_orientation_msg = imu_enu_msg->orientation;
            auto baselink_angular_msg = imu_enu_msg->angular_velocity;

            Eigen::Quaterniond enu_orientation;
            tf::quaternionMsgToEigen(enu_orientation_msg, enu_orientation);
            auto baselink_linear = ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

            //--------------- Generate Message Pointers ---------------//
            auto pose = boost::make_shared<geometry_msgs::PoseStamped>();
            auto twist = boost::make_shared<geometry_msgs::TwistStamped>();
            auto odom = boost::make_shared<nav_msgs::Odometry>();


            std_msgs::Header out_pose;
            out_pose.frame_id =  frame_id;
            //out_pose.stamp = vlp.timestamp;
            pose->header = out_pose; // TODO time sync need...  m_uas->synchronized_header(frame_id, vlp.timestamp);
            twist->header = pose->header;

            tf::pointEigenToMsg(enu_position, pose->pose.position);
            pose->pose.orientation = enu_orientation_msg;

            tf::vectorEigenToMsg(enu_velocity, twist->twist.linear);
            twist->twist.angular = baselink_angular_msg;


              odom->header.stamp = pose->header.stamp;
              odom->header.frame_id = tf_frame_id;
              odom->child_frame_id = tf_child_frame_id;
              tf::vectorEigenToMsg(baselink_linear, odom->twist.twist.linear);
              odom->twist.twist.angular = baselink_angular_msg;
              // reasonable defaults for covariance
              odom->pose.pose = pose->pose;

              for (int i=0; i< 3; i++) {
                  // linear velocity
                  odom->twist.covariance[i + 6*i] = 1e-4;
                  // angular velocity
                  odom->twist.covariance[(i + 3) + 6*(i + 3)] = 1e-4;
                  // position/ attitude
                  if (i==2) {
                    // z
                    odom->pose.covariance[i + 6*i] = 1e-6;
                    // yaw
                    odom->pose.covariance[(i + 3) + 6*(i + 3)] = 1e-6;
                  } else {
                    // x, y
                    odom->pose.covariance[i + 6*i] = 1e-6;
                    // roll, pitch
                    odom->pose.covariance[(i + 3) + 6*(i + 3)] = 1e-6;
                  }
              }

              //--------------- Publish Data ---------------//
              local_odom_pub.publish(odom);
              local_position_pub.publish(pose);
              local_velocity_pub.publish(twist);
              imu_data_pub.publish(imu_enu_msg);

        }

        void callbackVehicleAttitude(const px4_msgs::VehicleAttitudePtr msgs)
        {
            px4_msgs::VehicleAttitude va = *msgs;

            // ROS_INFO("Attitude: %f, %f, %f, %f",va.q[0], va.q[1], va.q[2], va.q[3]);    

            /** Orientation on the NED-aicraft frame:
             *  @snippet src/plugins/imu.cpp ned_aircraft_orient1
             */
            // [ned_aircraft_orient1]
            ned_aircraft_orientation = Eigen::Quaterniond(va.q[0], va.q[1], va.q[2], va.q[3]);
            // [ned_aircraft_orient1]

            /** The RPY describes the rotation: aircraft->NED.
             *  It is required to change this to aircraft->base_link:
             *  @snippet src/plugins/imu.cpp ned->baselink->enu
             */
            // [ned->baselink->enu]
            enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(ftf::transform_orientation_ned_enu(ned_aircraft_orientation));
            // [ned->baselink->enu]
        }

        void callbackVehicleAngularVelocity(const px4_msgs::VehicleAngularVelocityPtr msgs)
        {
           px4_msgs::VehicleAngularVelocity vav = *msgs;

            // ROS_INFO("Angular Velocity: %f, %f, %f", vav.xyz[0], vav.xyz[1], vav.xyz[1]);

            /** Angular velocity on the NED-aicraft frame:
             *  @snippet src/plugins/imu.cpp ned_ang_vel1
             */
            // [frd_ang_vel1]
            gyro_frd = Eigen::Vector3d(vav.xyz[0], vav.xyz[1], vav.xyz[1]);
            // [frd_ang_vel1]

            /** The angular velocity expressed in the aircraft frame.
             *  It is required to apply the static rotation to get it into the base_link frame:
             *  @snippet src/plugins/imu.cpp rotate_gyro
             */
            // [rotate_gyro]
            gyro_flu = ftf::transform_frame_aircraft_baselink(gyro_frd);
            // [rotate_gyro]
        }

      /**
       * @brief Setup 3x3 covariance matrix
       * @param cov		Covariance matrix
       * @param stdev		Standard deviation
       * @remarks		Diagonal computed from the stdev
       */
      void setup_covariance(ftf::Covariance3d &cov, double stdev)
      {
        ftf::EigenMapCovariance3d c(cov.data());
        c.setZero();
        if (stdev) {
          double sr = stdev * stdev;
          c.diagonal() << sr, sr, sr;
        }
        else {
          c(0,0) = -1.0;
        }
      }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleLocalPositionManagerNodelet, nodelet::Nodelet)
