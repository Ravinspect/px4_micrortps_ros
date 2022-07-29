#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/DebugKeyValue.h>

#include "debug_key_valuePublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class DebugKeyValueSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        DebugKeyValueSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~DebugKeyValueSubscriberNodelet()
        {
        }

    private:
    	debug_key_valuePublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/debug_key_value/in",5, &DebugKeyValueSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::DebugKeyValueConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::DebugKeyValueSubscriberNodelet, nodelet::Nodelet)
