#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/OffboardControlMode.h>

#include "mavlink_logSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class MavlinkLogPublisherNodelet : public nodelet::Nodelet
  {
    public:
        MavlinkLogPublisherNodelet()
        {

        }
    
        ~MavlinkLogPublisherNodelet()
        {
        }

    private:
    	mavlink_logSubscriber mysub;

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

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::MavlinkLogPublisherNodelet, nodelet::Nodelet)
