#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/OffboardControlMode.h>

#include "vehicle_local_positionSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleLocalPositionPublisherNodelet : public nodelet::Nodelet
  {
    public:
        VehicleLocalPositionPublisherNodelet()
        {

        }
    
        ~VehicleLocalPositionPublisherNodelet()
        {
        }

    private:
    	vehicle_local_positionSubscriber mysub;

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

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleLocalPositionPublisherNodelet, nodelet::Nodelet)
