#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "battery_statusSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class BatteryStatusPublisherNodelet : public nodelet::Nodelet
  {
    public:
        BatteryStatusPublisherNodelet()
        {

        }
    
        ~BatteryStatusPublisherNodelet()
        {
        }

    private:
    	battery_statusSubscriber mysub;

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

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::BatteryStatusPublisherNodelet, nodelet::Nodelet)
