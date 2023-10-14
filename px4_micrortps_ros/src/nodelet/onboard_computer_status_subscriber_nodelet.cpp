#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/OnboardComputerStatus.h>

#include "onboard_computer_statusPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class OnboardComputerStatusSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        OnboardComputerStatusSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~OnboardComputerStatusSubscriberNodelet()
        {
        }

    private:
    	onboard_computer_statusPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/onboard_computer_status/in",5, &OnboardComputerStatusSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::OnboardComputerStatusConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::OnboardComputerStatusSubscriberNodelet, nodelet::Nodelet)
