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
 * @file distance_sensorPublisher.cpp
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

#include "distance_sensorPublisher.h"
#include <ros/ros.h>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

distance_sensorPublisher::distance_sensorPublisher() : mp_participant(nullptr), mp_publisher(nullptr) {}

distance_sensorPublisher::~distance_sensorPublisher() {	Domain::removeParticipant(mp_participant);}

bool distance_sensorPublisher::init()
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
    Wparam.topic.topicName = "distance_sensorPubSubTopic";

    mp_publisher = Domain::createPublisher(mp_participant,Wparam,static_cast<PublisherListener*>(&m_listener));

    if(mp_publisher == nullptr)
    {
        return false;
    }

    std::cout << "Publisher created, waiting for Subscribers." << std::endl;
    return true;
}

void distance_sensorPublisher::PubListener::onPublicationMatched(Publisher* pub,MatchingInfo& info)
{
    (void)pub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Publisher matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Publisher unmatched" << std::endl;
    }
}

void distance_sensorPublisher::run(const px4_msgs::DistanceSensor& input)
{
    // Publication code
    distance_sensor st;

    st.timestamp_(input.timestamp);
    st.device_id_(input.device_id);
    st.min_distance_(input.min_distance);
    st.max_distance_(input.max_distance);
    st.current_distance_(input.current_distance);
    st.variance_(input.variance);
    st.signal_quality_(input.signal_quality);
    st.type_(input.type);  
    
    //TODO h_fov, v_fov & q is missing.  
    
    st.orientation_(input.orientation);



    mp_publisher->write(&st);
}
