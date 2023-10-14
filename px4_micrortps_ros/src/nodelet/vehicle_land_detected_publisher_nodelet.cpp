#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "vehicle_land_detectedSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleLandDetectedPublisherNodelet : public nodelet::Nodelet
  {
    public:
        VehicleLandDetectedPublisherNodelet()
        {

        }
    
        ~VehicleLandDetectedPublisherNodelet()
        {
        }

    private:
    	vehicle_land_detectedSubscriber mysub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();

            if (mysub.init(private_nh))
            {
                //TODO hata mesajı yazılacak..
            }

            
            NODELET_DEBUG("Initialized the Nodelet");
        }

  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleLandDetectedPublisherNodelet, nodelet::Nodelet)
