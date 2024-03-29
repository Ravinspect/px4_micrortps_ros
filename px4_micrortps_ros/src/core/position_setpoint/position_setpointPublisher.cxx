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
 * @file position_setpointPublisher.cpp
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

#include "position_setpointPublisher.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

position_setpointPublisher::position_setpointPublisher() : mp_participant(nullptr), mp_publisher(nullptr) {}

position_setpointPublisher::~position_setpointPublisher() {	Domain::removeParticipant(mp_participant);}

bool position_setpointPublisher::init()
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
    Wparam.topic.topicName = "position_setpointPubSubTopic";

    mp_publisher = Domain::createPublisher(mp_participant,Wparam,static_cast<PublisherListener*>(&m_listener));

    if(mp_publisher == nullptr)
    {
        return false;
    }

    std::cout << "Publisher created, waiting for Subscribers." << std::endl;
    return true;
}

void position_setpointPublisher::PubListener::onPublicationMatched(Publisher* pub,MatchingInfo& info)
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

void position_setpointPublisher::run(const px4_msgs::PositionSetpoint& input)
{
    //next
    position_setpoint st;
    st.timestamp_(input.timestamp);
    st.valid_(input.valid);
    st.type_(input.type);

    st.vx_(input.vx);
    st.vy_(input.vy);
    st.vz_(input.vz);

    st.velocity_valid_(input.velocity_valid);
    st.velocity_frame_(input.velocity_frame);
    st.alt_valid_(input.alt_valid);

    st.lat_(input.lat);
    st.lon_(input.lon);
    st.alt_(input.alt);
    st.yaw_(input.yaw);
    st.yaw_valid_(input.yaw_valid);

    st.yawspeed_(input.yawspeed);
    st.yawspeed_valid_(input.yawspeed_valid);

    st.landing_gear_(input.landing_gear);

    st.loiter_radius_(input.loiter_radius);

    st.loiter_direction_(input.loiter_direction);

    st.acceptance_radius_(input.acceptance_radius);

    st.cruising_speed_(input.cruising_speed);

    st.cruising_throttle_(input.cruising_throttle);

    st.disable_weather_vane_(input.disable_weather_vane);

    mp_publisher->write(&st);
}
