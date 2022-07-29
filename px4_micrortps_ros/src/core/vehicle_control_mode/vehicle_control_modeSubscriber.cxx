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
 * @file vehicle_control_modeSubscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "vehicle_control_modeSubscriber.h"

#include "ros/ros.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

vehicle_control_modeSubscriber::vehicle_control_modeSubscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

vehicle_control_modeSubscriber::~vehicle_control_modeSubscriber() {	Domain::removeParticipant(mp_participant);}

bool vehicle_control_modeSubscriber::init(const ros::NodeHandle& private_nh)
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
    Rparam.topic.topicName = "vehicle_control_modePubSubTopic";
    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
    {
        return false;
    }

    m_listener.nh_ = private_nh;
    m_listener.vehicle_control_mode_publisher_ = m_listener.nh_.advertise<px4_msgs::VehicleControlMode>("/px4_micrortps_ros/vehicle_control_mode/out", 10);

    return true;
}

void vehicle_control_modeSubscriber::SubListener::onSubscriptionMatched(Subscriber* sub,MatchingInfo& info)
{
    (void)sub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber vehicle_control_mode matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber vehicle_control_mode unmatched" << std::endl;
    }
}

void vehicle_control_modeSubscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    // Take data
    vehicle_control_mode st;

    if(sub->takeNextData(&st, &m_info))
    {
        if(m_info.sampleKind == ALIVE)
        {
            px4_msgs::VehicleControlModePtr msg_ptr(new px4_msgs::VehicleControlMode);
            convertUORBToPX4Message(st, msg_ptr); 

            vehicle_control_mode_publisher_.publish(msg_ptr);
        }
    }
}

void vehicle_control_modeSubscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}

void vehicle_control_modeSubscriber::SubListener::convertUORBToPX4Message(const vehicle_control_mode& uorb_msg,  px4_msgs::VehicleControlModePtr& px4_msg)
{
    px4_msg->timestamp = uorb_msg.timestamp_();
    px4_msg->flag_armed = uorb_msg.flag_armed_();
    px4_msg->flag_multicopter_position_control_enabled = uorb_msg.flag_multicopter_position_control_enabled_();
    px4_msg->flag_control_manual_enabled = uorb_msg.flag_control_manual_enabled_();
    px4_msg->flag_control_auto_enabled = uorb_msg.flag_control_auto_enabled_();
    px4_msg->flag_control_offboard_enabled = uorb_msg.flag_control_offboard_enabled_();
    px4_msg->flag_control_rates_enabled = uorb_msg.flag_control_rates_enabled_();
    px4_msg->flag_control_attitude_enabled = uorb_msg.flag_control_attitude_enabled_();
    px4_msg->flag_control_acceleration_enabled = uorb_msg.flag_control_acceleration_enabled_();
    px4_msg->flag_control_velocity_enabled = uorb_msg.flag_control_velocity_enabled_();
    px4_msg->flag_control_position_enabled = uorb_msg.flag_control_position_enabled_();
    px4_msg->flag_control_altitude_enabled = uorb_msg.flag_control_altitude_enabled_();
    px4_msg->flag_control_climb_rate_enabled = uorb_msg.flag_control_climb_rate_enabled_();
    px4_msg->flag_control_termination_enabled = uorb_msg.flag_control_termination_enabled_();

}