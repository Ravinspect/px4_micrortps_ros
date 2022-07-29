#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "vehicle_statusSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleStatusPublisherNodelet : public nodelet::Nodelet
  {
    public:
        VehicleStatusPublisherNodelet()
        {

        }
    
        ~VehicleStatusPublisherNodelet()
        {
        }

    private:
    	vehicle_statusSubscriber mysub;

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

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleStatusPublisherNodelet, nodelet::Nodelet)
