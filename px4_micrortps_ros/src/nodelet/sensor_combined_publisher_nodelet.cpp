#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "sensor_combinedSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class SensorCombinedPublisherNodelet : public nodelet::Nodelet
  {
    public:
        SensorCombinedPublisherNodelet()
        {

        }
    
        ~SensorCombinedPublisherNodelet()
        {
        }

    private:
    	sensor_combinedSubscriber mysub;

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

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::SensorCombinedPublisherNodelet, nodelet::Nodelet)
