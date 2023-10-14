#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/VehicleCommand.h>

#include "vehicle_commandPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleCommandSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        VehicleCommandSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~VehicleCommandSubscriberNodelet()
        {
        }

    private:
    	vehicle_commandPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/vehicle_command/in",5, &VehicleCommandSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::VehicleCommandConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleCommandSubscriberNodelet, nodelet::Nodelet)
