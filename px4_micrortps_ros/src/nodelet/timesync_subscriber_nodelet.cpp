#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/Timesync.h>

#include "timesyncPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class TimesyncSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        TimesyncSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~TimesyncSubscriberNodelet()
        {
        }

    private:
    	timesyncPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/timesync/in",5, &TimesyncSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::TimesyncConstPtr input)
        {
            ROS_INFO("Received a message TimesyncSubscriberNodelet input = %llu", input->tc1);

            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::TimesyncSubscriberNodelet, nodelet::Nodelet)
