#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "collision_constraintsSubscriber.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class CollisionConstraintsPublisherNodelet : public nodelet::Nodelet
  {
    public:
        CollisionConstraintsPublisherNodelet()
        {

        }
    
        ~CollisionConstraintsPublisherNodelet()
        {
        }

    private:
    	collision_constraintsSubscriber mysub;

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

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::CollisionConstraintsPublisherNodelet, nodelet::Nodelet)
