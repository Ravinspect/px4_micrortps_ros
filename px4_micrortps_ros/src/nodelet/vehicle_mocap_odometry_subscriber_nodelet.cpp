#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/VehicleMocapOdometry.h>

#include "vehicle_mocap_odometryPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleMocapOdometrySubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        VehicleMocapOdometrySubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~VehicleMocapOdometrySubscriberNodelet()
        {
        }

    private:
    	vehicle_mocap_odometryPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/vehicle_mocap_odometry/in",5, &VehicleMocapOdometrySubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::VehicleMocapOdometryConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleMocapOdometrySubscriberNodelet, nodelet::Nodelet)
