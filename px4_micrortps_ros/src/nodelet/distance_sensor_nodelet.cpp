#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "distance_sensorSubscriber.h"
#include "distance_sensorPublisher.h"


#include <fastrtps/Domain.h>


namespace px4_micrortps_ros
{

  class DistanceSensorPublisherNodelet : public nodelet::Nodelet
  {
    public:
        DistanceSensorPublisherNodelet()
        {
            if (mypub.init())
            {
                //TODO hata mesajı yazılacak..
            }
        }
    
        ~DistanceSensorPublisherNodelet()
        {
        }

    private:
        ros::Subscriber sub;

    	distance_sensorSubscriber mysub;
        distance_sensorPublisher mypub;

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();

            if (mysub.init(private_nh))
            {
                //TODO hata mesajı yazılacak..
            }

            sub = private_nh.subscribe("/px4_micrortps_ros/distance_sensor/in",5, &DistanceSensorPublisherNodelet::callback, this);
            NODELET_DEBUG("Initialized the Nodelet");
        }


        void callback(const px4_msgs::DistanceSensorPtr input)
        {
            std::cout << "DistanceSensorPtr 2:" << input << std::endl;
            mypub.run(*input);
        }

  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::DistanceSensorPublisherNodelet, nodelet::Nodelet)
