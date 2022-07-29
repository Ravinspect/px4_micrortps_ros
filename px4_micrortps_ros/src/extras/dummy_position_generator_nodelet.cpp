#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <thread>
#include <chrono>

#include <array>

#include <geometry_msgs/TransformStamped.h>

#include <thread>
#include <chrono>

#include <array>
#include <ros/assert.h>

// for Covariance types
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>


#include  <tf/transform_broadcaster.h>

#include <eigen_conversions/eigen_msg.h>
#include <mavros/utils.h>
#include <mavros/frame_tf.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry> 


using namespace mavros;

namespace px4_micrortps_ros
{
  class DummyPositionGeneratorNodelet : public nodelet::Nodelet
  {
    public:
         ros::Publisher pub;
         int counter = 0;

        DummyPositionGeneratorNodelet()
        {

        }
    
        ~DummyPositionGeneratorNodelet()
        {
        }
    private: 
        ros::Time last_transform_stamp;
        geometry_msgs::TransformStamped transform_stamped;    

    private:

        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            NODELET_DEBUG("Initialized the Nodelet");

            pub = private_nh.advertise<geometry_msgs::TransformStamped>("/position_sent", 10);

            geometry_msgs::TransformStampedPtr transform_stamped(new geometry_msgs::TransformStamped);

            float x = 0.0;
            float y = 0.0;
            float z = 0.0;

            while(counter < 1000001)
            {
                ros::Time last_transform_stamp;

                transform_stamped->header.stamp = ros::Time::now();
                transform_stamped->transform.translation.x = x + 0.5;
                transform_stamped->transform.translation.y = y + 0.5;
                transform_stamped->transform.translation.z = z + 0.5;

                

                if(counter == 1000000)
                {
                    std::cout << "dummy x: " <<  transform_stamped->transform.translation.x 
                                    <<  ", y:" << transform_stamped->transform.translation.y 
                                    <<  ", z :"<< transform_stamped->transform.translation.z << std::endl;
                    pub.publish(transform_stamped);
                    counter = 0;    
                }    

                x = x + 0.5;
                y = y + 0.5;
                z = z + 0.5;

                counter ++;
            }
        }
  };
}

PLUGINLIB_EXPORT_CLASS( px4_micrortps_ros::DummyPositionGeneratorNodelet, nodelet::Nodelet)
