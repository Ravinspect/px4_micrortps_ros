#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/TelemetryStatus.h>

#include "telemetry_statusPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class TelemetryStatusSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        TelemetryStatusSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~TelemetryStatusSubscriberNodelet()
        {
        }

    private:
    	telemetry_statusPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/telemetry_statu/in",5, &TelemetryStatusSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::TelemetryStatusConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::TelemetryStatusSubscriberNodelet, nodelet::Nodelet)
