#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/VehicleVisualOdometry.h>

#include "vehicle_visual_odometryPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleVisualOdometrySubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        VehicleVisualOdometrySubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~VehicleVisualOdometrySubscriberNodelet()
        {
        }

    private:
    	vehicle_visual_odometryPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/vehicle_visual_odometry/in",5, &VehicleVisualOdometrySubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::VehicleVisualOdometryPtr input)
        {
            std::cout << "VehicleVisualOdometryPtr 2:" << input << std::endl;
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleVisualOdometrySubscriberNodelet, nodelet::Nodelet)
