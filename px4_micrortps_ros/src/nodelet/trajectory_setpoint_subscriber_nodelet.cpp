#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/TrajectorySetpoint.h>

#include "trajectory_setpointPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class TrajectorySetpointSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        TrajectorySetpointSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~TrajectorySetpointSubscriberNodelet()
        {
        }

    private:
    	trajectory_setpointPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/trajectory_setpoint/in",5, &TrajectorySetpointSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::TrajectorySetpointConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::TrajectorySetpointSubscriberNodelet, nodelet::Nodelet)
