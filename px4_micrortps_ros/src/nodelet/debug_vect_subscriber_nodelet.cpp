#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <px4_msgs/DebugVect.h>

#include "debug_vectPublisher.h"
#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class DebugVectSubscriberNodelet : public nodelet::Nodelet
  {
    public:
         ros::Subscriber sub;

        DebugVectSubscriberNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~DebugVectSubscriberNodelet()
        {
        }

    private:
    	debug_vectPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");
            sub = private_nh.subscribe("/px4_micrortps_ros/debug_vect/in",5, &DebugVectSubscriberNodelet::callback, this);
        }
        
        void callback(const px4_msgs::DebugVectConstPtr input)
        {
            mypub.run(*input);
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::DebugVectSubscriberNodelet, nodelet::Nodelet)
