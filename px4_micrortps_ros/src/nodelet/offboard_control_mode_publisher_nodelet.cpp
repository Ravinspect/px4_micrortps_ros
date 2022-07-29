#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/OffboardControlMode.h>

#include "offboard_control_modePublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class OffboardControlModePublisherNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        OffboardControlModePublisherNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~OffboardControlModePublisherNodelet()
        {
        }

    private:
    	offboard_control_modePublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/offboard_control_mode/in",5, &OffboardControlModePublisherNodelet::callback, this);
        }
        
        void callback(const px4_msgs::OffboardControlModeConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::OffboardControlModePublisherNodelet, nodelet::Nodelet)
