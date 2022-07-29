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
 * @file vehicle_attitudeSubscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "vehicle_attitudeSubscriber.h"
#include <px4_msgs/VehicleAttitude.h>
#include <ros/ros.h>


using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

vehicle_attitudeSubscriber::vehicle_attitudeSubscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

vehicle_attitudeSubscriber::~vehicle_attitudeSubscriber() {	Domain::removeParticipant(mp_participant);}

bool vehicle_attitudeSubscriber::init(const ros::NodeHandle& private_nh)
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
    Rparam.topic.topicName = "vehicle_attitudePubSubTopic";
    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
    {
        return false;
    }

    m_listener.nh_ = private_nh;
    m_listener.vehicle_attitude_publisher_ = m_listener.nh_.advertise<px4_msgs::VehicleAttitude>("/px4_micrortps_ros/vehicle_attitude/out", 10);

    return true;
}

void vehicle_attitudeSubscriber::SubListener::onSubscriptionMatched(Subscriber* sub,MatchingInfo& info)
{
    (void)sub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber vehicle_attitude  matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber vehicle_attitude unmatched" << std::endl;
    }
}

void vehicle_attitudeSubscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    // Take data
    vehicle_attitude st;

    if(sub->takeNextData(&st, &m_info))
    {
        if(m_info.sampleKind == ALIVE)
        {
            // Print your structure data here.
            px4_msgs::VehicleAttitudePtr msg_ptr(new px4_msgs::VehicleAttitude);
            convertUORBToPX4Message(st, msg_ptr); 
            //publish message
            vehicle_attitude_publisher_.publish(msg_ptr);
        }
    }
}

void vehicle_attitudeSubscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}

void vehicle_attitudeSubscriber::SubListener::convertUORBToPX4Message(const vehicle_attitude &st, px4_msgs::VehicleAttitudePtr &msg_ptr)
{
    msg_ptr->timestamp = st.timestamp_();
    msg_ptr->timestamp_sample = st.timestamp_sample_();
    msg_ptr->q[0] = st.q()[0];
    msg_ptr->q[1] = st.q()[1];
    msg_ptr->q[2] = st.q()[2];
    msg_ptr->q[3] = st.q()[3];

    msg_ptr->delta_q_reset[0] = st.delta_q_reset()[0];
    msg_ptr->delta_q_reset[1] = st.delta_q_reset()[1];
    msg_ptr->delta_q_reset[2] = st.delta_q_reset()[2];
    msg_ptr->delta_q_reset[3] = st.delta_q_reset()[3];
    msg_ptr->quat_reset_counter = st.quat_reset_counter_();
}


