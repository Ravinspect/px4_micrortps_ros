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
 * @file vehicle_statusSubscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "vehicle_statusSubscriber.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

vehicle_statusSubscriber::vehicle_statusSubscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

vehicle_statusSubscriber::~vehicle_statusSubscriber() {	Domain::removeParticipant(mp_participant);}

bool vehicle_statusSubscriber::init(const ros::NodeHandle& private_nh)
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
    Rparam.topic.topicName = "vehicle_statusPubSubTopic";
    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
    {
        return false;
    }

    m_listener.nh_ = private_nh;
    m_listener.vehicle_status_publisher_ = m_listener.nh_.advertise<px4_msgs::VehicleStatus>("/px4_micrortps_ros/vehicle_status/out", 10);

    return true;
}

void vehicle_statusSubscriber::SubListener::onSubscriptionMatched(Subscriber* sub,MatchingInfo& info)
{
    (void)sub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber vehicle_status matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber vehicle_status unmatched" << std::endl;
    }
}

void vehicle_statusSubscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    // Take data
    vehicle_status st;

    if(sub->takeNextData(&st, &m_info))
    {
        if(m_info.sampleKind == ALIVE)
        {
            px4_msgs::VehicleStatusPtr msg_ptr(new px4_msgs::VehicleStatus);
            convertUORBToPX4Message(st, msg_ptr); 
            // Publish message
            vehicle_status_publisher_.publish(msg_ptr);
        }
    }
}

void vehicle_statusSubscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}

void vehicle_statusSubscriber::SubListener::convertUORBToPX4Message(const vehicle_status& uorb_msg, px4_msgs::VehicleStatusPtr& px4_msg)
{
    px4_msg->timestamp = uorb_msg.timestamp_();
    px4_msg->nav_state = uorb_msg.nav_state_();
    px4_msg->nav_state_timestamp = uorb_msg.nav_state_timestamp_();
    px4_msg->arming_state = uorb_msg.arming_state_();
    px4_msg->hil_state = uorb_msg.hil_state_();
    px4_msg->failsafe = uorb_msg.failsafe_();
    px4_msg->failsafe_timestamp = uorb_msg.failsafe_timestamp_();

    px4_msg->system_type = uorb_msg.system_type_();
    px4_msg->system_id = uorb_msg.system_id_();
    px4_msg->component_id = uorb_msg.component_id_();

    px4_msg->vehicle_type = uorb_msg.vehicle_type_();

    px4_msg->is_vtol = uorb_msg.is_vtol_();
    px4_msg->is_vtol_tailsitter = uorb_msg.is_vtol_tailsitter_();
    // TODO does not exist in v1.12.3
    // px4_msg->vtol_fw_permanent_stab = uorb_msg.vtol_fw_permanent_stab_();
    px4_msg->in_transition_mode = uorb_msg.in_transition_mode_();
    px4_msg->in_transition_to_fw = uorb_msg.in_transition_to_fw_();

    px4_msg->rc_signal_lost = uorb_msg.rc_signal_lost_();
    px4_msg->data_link_lost = uorb_msg.data_link_lost_();
    px4_msg->data_link_lost_counter = uorb_msg.data_link_lost_counter_();

    px4_msg->high_latency_data_link_lost = uorb_msg.high_latency_data_link_lost_();

    px4_msg->mission_failure = uorb_msg.mission_failure_();
    px4_msg->geofence_violated = uorb_msg.geofence_violated_();

    px4_msg->failure_detector_status = uorb_msg.failure_detector_status_();

    px4_msg->onboard_control_sensors_present = uorb_msg.onboard_control_sensors_present_();
    px4_msg->onboard_control_sensors_enabled = uorb_msg.onboard_control_sensors_enabled_();
    px4_msg->onboard_control_sensors_health = uorb_msg.onboard_control_sensors_health_();


    px4_msg->latest_arming_reason = uorb_msg.latest_arming_reason_();
    px4_msg->latest_disarming_reason = uorb_msg.latest_disarming_reason_();

    px4_msg->armed_time = uorb_msg.armed_time_();

    px4_msg->takeoff_time = uorb_msg.takeoff_time_();
}