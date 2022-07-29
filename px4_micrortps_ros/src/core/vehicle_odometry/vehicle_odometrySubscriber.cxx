// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file vehicle_odometrySubscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "vehicle_odometrySubscriber.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

vehicle_odometrySubscriber::vehicle_odometrySubscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

vehicle_odometrySubscriber::~vehicle_odometrySubscriber() {	Domain::removeParticipant(mp_participant);}

bool vehicle_odometrySubscriber::init(const ros::NodeHandle& private_nh)
{
    // Create RTPSParticipant

    ParticipantAttributes PParam;
    PParam.rtps.setName("Participant_subscriber"); //You can put the name you want
    mp_participant = Domain::createParticipant(PParam);
    if(mp_participant == nullptr)
    {
        return false;
    }

    //Register the type

    Domain::registerType(mp_participant, static_cast<TopicDataType*>(&myType));

    // Create Subscriber

    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = myType.getName(); //Must be registered before the creation of the subscriber
    Rparam.topic.topicName = "vehicle_odometryPubSubTopic";
    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
    {
        return false;
    }

    m_listener.nh_ = private_nh;
    m_listener.vehicle_odometry_publisher_ = m_listener.nh_.advertise<px4_msgs::VehicleOdometry>("/px4_micrortps_ros/vehicle_odometry/out", 10);

    return true;
}

void vehicle_odometrySubscriber::SubListener::onSubscriptionMatched(Subscriber* sub,MatchingInfo& info)
{
    (void)sub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber vehicle_odometry matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber vehicle_odometry unmatched" << std::endl;
    }
}

void vehicle_odometrySubscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    // Take data
    vehicle_odometry st;

    if(sub->takeNextData(&st, &m_info))
    {
        if(m_info.sampleKind == ALIVE)
        {
            px4_msgs::VehicleOdometryPtr msg_ptr(new px4_msgs::VehicleOdometry);
            convertUORBToPX4Message(st, msg_ptr); 

            vehicle_odometry_publisher_.publish(msg_ptr);
        }
    }
}

void vehicle_odometrySubscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}

void vehicle_odometrySubscriber::SubListener::convertUORBToPX4Message(const vehicle_odometry& uorb_msg,  px4_msgs::VehicleOdometryPtr& px4_msg_ptr)
{
    px4_msg_ptr->timestamp = uorb_msg.timestamp_();
    px4_msg_ptr->timestamp_sample = uorb_msg.timestamp_sample_();

    px4_msg_ptr->local_frame = uorb_msg.local_frame_();

    px4_msg_ptr->x = uorb_msg.x_();
    px4_msg_ptr->y = uorb_msg.y_();
    px4_msg_ptr->z = uorb_msg.z_();

    px4_msg_ptr->q[0] = uorb_msg.q()[0];
    px4_msg_ptr->q[1] = uorb_msg.q()[1];
    px4_msg_ptr->q[2] = uorb_msg.q()[2];
    px4_msg_ptr->q[3] = uorb_msg.q()[3];

    px4_msg_ptr->q_offset[0] = uorb_msg.q_offset()[0];
    px4_msg_ptr->q_offset[1] = uorb_msg.q_offset()[1];
    px4_msg_ptr->q_offset[2] = uorb_msg.q_offset()[2];    
    px4_msg_ptr->q_offset[3] = uorb_msg.q_offset()[3];

    for(int i = 0; i < uorb_msg.pose_covariance().size(); i++)
    {
        px4_msg_ptr->pose_covariance[i] = uorb_msg.pose_covariance()[i];
    }

    px4_msg_ptr->velocity_frame = uorb_msg.velocity_frame_();

    px4_msg_ptr->vx = uorb_msg.vx_();
    px4_msg_ptr->vy = uorb_msg.vy_();
    px4_msg_ptr->vz = uorb_msg.vz_();

    px4_msg_ptr->rollspeed = uorb_msg.rollspeed_();
    px4_msg_ptr->pitchspeed = uorb_msg.pitchspeed_();
    px4_msg_ptr->yawspeed = uorb_msg.yawspeed_();

    for(int i = 0; i < uorb_msg.velocity_covariance().size(); i++)
    {
        px4_msg_ptr->velocity_covariance[i] = uorb_msg.velocity_covariance()[i];
    }

    // TODO v.1.13 in new versions
    //px4_msg_ptr->reset_counter = uorb_msg.reset_counter_();
}