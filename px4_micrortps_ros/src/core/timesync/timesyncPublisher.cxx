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
 * @file timesyncPublisher.cpp
 * This file contains the implementation of the publisher functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/attributes/PublisherAttributes.h>

#include <fastrtps/Domain.h>

#include <thread>
#include <chrono>

#include "timesyncPublisher.h"

#include <ros/ros.h>


using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

timesyncPublisher::timesyncPublisher() : mp_participant(nullptr), mp_publisher(nullptr) {}

timesyncPublisher::~timesyncPublisher() {	Domain::removeParticipant(mp_participant);}

bool timesyncPublisher::init()
{
    // Create RTPSParticipant

    ParticipantAttributes PParam;
    PParam.rtps.setName("Participant_publisher");  //You can put here the name you want
    mp_participant = Domain::createParticipant(PParam);
    if(mp_participant == nullptr)
    {
        return false;
    }

    //Register the type

    Domain::registerType(mp_participant, static_cast<TopicDataType*>(&myType));

    // Create Publisher

    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = myType.getName();  //This type MUST be registered
    Wparam.topic.topicName = "timesyncPubSubTopic";

    mp_publisher = Domain::createPublisher(mp_participant,Wparam,static_cast<PublisherListener*>(&m_listener));

    if(mp_publisher == nullptr)
    {
        return false;
    }

    std::cout << "Publisher created, waiting for Subscribers." << std::endl;
    return true;
}

void timesyncPublisher::PubListener::onPublicationMatched(Publisher* pub,MatchingInfo& info)
{
    (void)pub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "timesync Publisher matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Publisher unmatched" << std::endl;
    }
}

void timesyncPublisher::run(const px4_msgs::Timesync& input)
{

    // ROS_INFO("Sending message timesyncPublisher");

    //next
    timesync st;
    st.timestamp_(input.timestamp);
    st.seq_(input.seq);
    st.tc1_(input.tc1);
    st.ts1_(input.ts1);

    st.timestamp_(input.timestamp);
}
