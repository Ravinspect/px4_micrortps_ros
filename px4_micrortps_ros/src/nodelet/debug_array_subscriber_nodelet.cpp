#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/DebugArray.h>

#include "debug_arrayPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class DebugArraySubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        DebugArraySubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~DebugArraySubscriberNodelet()
        {
        }

    private:
    	debug_arrayPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the DebugArray Subscriber Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/debug_array/in",5, &DebugArraySubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::DebugArrayConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::DebugArraySubscriberNodelet, nodelet::Nodelet)
