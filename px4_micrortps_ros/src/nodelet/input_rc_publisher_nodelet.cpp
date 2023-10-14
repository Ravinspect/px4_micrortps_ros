#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "input_rcSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class InputRCPublisherNodelet : public nodelet::Nodelet
  {
    public:
        InputRCPublisherNodelet()
        {

        }
    
        ~InputRCPublisherNodelet()
        {
        }

    private:
    	input_rcSubscriber mysub;

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

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::InputRCPublisherNodelet, nodelet::Nodelet)
