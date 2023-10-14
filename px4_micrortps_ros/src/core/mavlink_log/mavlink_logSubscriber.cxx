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
 * @file mavlink_logSubscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <stdio.h>
#include <string.h>

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "mavlink_logSubscriber.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

mavlink_logSubscriber::mavlink_logSubscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

mavlink_logSubscriber::~mavlink_logSubscriber() {	Domain::removeParticipant(mp_participant);}

bool mavlink_logSubscriber::init(const ros::NodeHandle& private_nh)
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
    Rparam.topic.topicName = "mavlink_logPubSubTopic";
    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
    {
        return false;
    }

    m_listener.nh_ = private_nh;
    m_listener.mavlink_log_publisher_ = m_listener.nh_.advertise<px4_msgs::MavlinkLog>("/px4_micrortps_ros/mavlink_log/out", 10);

    return true;
}

void mavlink_logSubscriber::SubListener::onSubscriptionMatched(Subscriber* sub,MatchingInfo& info)
{
    (void)sub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber mavlink_log matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber mavlink_log unmatched" << std::endl;
    }
}

void mavlink_logSubscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    // Take data
    mavlink_log st;

    if(sub->takeNextData(&st, &m_info))
    {
        if(m_info.sampleKind == ALIVE)
        {
            px4_msgs::MavlinkLogPtr msg_ptr(new px4_msgs::MavlinkLog);
            convertUORBToPX4Message(st, msg_ptr); 

            mavlink_log_publisher_.publish(msg_ptr);
        }
    }
}

void mavlink_logSubscriber::SubListener::convertUORBToPX4Message(const mavlink_log &st, px4_msgs::MavlinkLogPtr & px4_msg_ptr)
{
    px4_msg_ptr->timestamp = st.timestamp_();
    
    px4_msg_ptr->severity = st.severity_();

    ROS_INFO("Mavlink_Log Message Step1");

    std::string new_message = std::string(st.text().data(), strnlen(st.text().data(), st.text().size()));

    std::cout << "new log message: " << new_message << std::endl;
    boost::array<unsigned char, 127> text;
    for(int i= 0; i < 127;i++)
    {
        text[i] = new_message[i];
    }

    px4_msg_ptr->text =  text;
}



void mavlink_logSubscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}

