#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "vehicle_angular_velocitySubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleAngularVelocityPublisherNodelet : public nodelet::Nodelet
  {
    public:
        VehicleAngularVelocityPublisherNodelet()
        {

        }
    
        ~VehicleAngularVelocityPublisherNodelet()
        {
        }

    private:
    	vehicle_angular_velocitySubscriber mysub;

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

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleAngularVelocityPublisherNodelet, nodelet::Nodelet)
