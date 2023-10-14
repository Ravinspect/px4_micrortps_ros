#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "vehicle_trajectory_waypoint_desiredSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class VehicleTrajectoryWaypointDesiredPublisherNodelet : public nodelet::Nodelet
  {
    public:
        VehicleTrajectoryWaypointDesiredPublisherNodelet()
        {

        }
    
        ~VehicleTrajectoryWaypointDesiredPublisherNodelet()
        {
        }

    private:
    	vehicle_trajectory_waypoint_desiredSubscriber mysub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();

            if (mysub.init(private_nh))
            {
                //TODO hata mesajı yazılacak..
            }

            
            NODELET_DEBUG("Initialized the Nodelet");
        }

  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::VehicleTrajectoryWaypointDesiredPublisherNodelet, nodelet::Nodelet)
