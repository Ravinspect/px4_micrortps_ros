#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/VehicleTrajectoryBezier.h>

#include "vehicle_trajectory_bezierPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleTrajectoryBezierSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        VehicleTrajectoryBezierSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~VehicleTrajectoryBezierSubscriberNodelet()
        {
        }

    private:
    	vehicle_trajectory_bezierPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/vehicle_trajectory_bezier/in",5, &VehicleTrajectoryBezierSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::VehicleTrajectoryBezierConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleTrajectoryBezierSubscriberNodelet, nodelet::Nodelet)
