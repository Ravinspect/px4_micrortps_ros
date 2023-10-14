#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/VehicleLocalPositionSetpoint.h>

#include "vehicle_local_position_setpointPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleLocalPositionSetpointSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        VehicleLocalPositionSetpointSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~VehicleLocalPositionSetpointSubscriberNodelet()
        {
        }

    private:
    	vehicle_local_position_setpointPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/vehicle_local_position_setpoint/in",5, &VehicleLocalPositionSetpointSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::VehicleLocalPositionSetpointConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleLocalPositionSetpointSubscriberNodelet, nodelet::Nodelet)
