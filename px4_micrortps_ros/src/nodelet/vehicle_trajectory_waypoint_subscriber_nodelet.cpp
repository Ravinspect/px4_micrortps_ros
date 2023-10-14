#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/VehicleTrajectoryWaypoint.h>

#include "vehicle_trajectory_waypointPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleTrajectoryWaypointSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        VehicleTrajectoryWaypointSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~VehicleTrajectoryWaypointSubscriberNodelet()
        {
        }

    private:
    	vehicle_trajectory_waypointPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/vehicle_trajectory_waypoint",5, &VehicleTrajectoryWaypointSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::VehicleTrajectoryWaypointConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleTrajectoryWaypointSubscriberNodelet, nodelet::Nodelet)
