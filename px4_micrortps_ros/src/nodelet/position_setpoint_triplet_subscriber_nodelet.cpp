#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/PositionSetpointTriplet.h>

#include "position_setpoint_tripletPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class PositionSetpointTripletSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        PositionSetpointTripletSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~PositionSetpointTripletSubscriberNodelet()
        {
        }

    private:
    	position_setpoint_tripletPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/position_setpoint_triplet/in",5, &PositionSetpointTripletSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::PositionSetpointTripletConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::PositionSetpointTripletSubscriberNodelet, nodelet::Nodelet)
