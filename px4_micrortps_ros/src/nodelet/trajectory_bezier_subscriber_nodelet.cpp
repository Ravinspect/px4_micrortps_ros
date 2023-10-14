#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/TrajectoryBezier.h>

#include "trajectory_bezierPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class TrajectoryBezierSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        TrajectoryBezierSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~TrajectoryBezierSubscriberNodelet()
        {
        }

    private:
    	trajectory_bezierPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/trajectory_bezier/in",5, &TrajectoryBezierSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::TrajectoryBezierConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::TrajectoryBezierSubscriberNodelet, nodelet::Nodelet)
