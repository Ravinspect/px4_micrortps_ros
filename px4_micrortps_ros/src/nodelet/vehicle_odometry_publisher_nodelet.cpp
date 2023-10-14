#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "vehicle_odometrySubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleOdometryPublisherNodelet : public nodelet::Nodelet
  {
    public:
        VehicleOdometryPublisherNodelet()
        {

        }
    
        ~VehicleOdometryPublisherNodelet()
        {
        }

    private:
    	vehicle_odometrySubscriber mysub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getMTPrivateNodeHandle();

            if (mysub.init(private_nh))
            {
                //TODO hata mesajı yazılacak..
            }

            
            NODELET_DEBUG("Initialized the Nodelet");
        }

  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleOdometryPublisherNodelet, nodelet::Nodelet)
