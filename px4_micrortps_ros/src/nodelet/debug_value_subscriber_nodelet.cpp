#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/DebugValue.h>

#include "debug_valuePublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class DebugValueSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        DebugValueSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~DebugValueSubscriberNodelet()
        {
        }

    private:
    	debug_valuePublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/debug_value/in",5, &DebugValueSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::DebugValueConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::DebugValueSubscriberNodelet, nodelet::Nodelet)
