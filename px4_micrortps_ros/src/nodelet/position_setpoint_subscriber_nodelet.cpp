#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/PositionSetpoint.h>

#include "position_setpointPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class PositionSetpointSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        PositionSetpointSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~PositionSetpointSubscriberNodelet()
        {
        }

    private:
    	position_setpointPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/position_setpoint/in",5, &PositionSetpointSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::PositionSetpointConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::PositionSetpointSubscriberNodelet, nodelet::Nodelet)
