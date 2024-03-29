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
 * @file vehicle_trajectory_waypointPublisher.cpp
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

#include "vehicle_trajectory_waypointPublisher.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

vehicle_trajectory_waypointPublisher::vehicle_trajectory_waypointPublisher() : mp_participant(nullptr), mp_publisher(nullptr) {}

vehicle_trajectory_waypointPublisher::~vehicle_trajectory_waypointPublisher() {	Domain::removeParticipant(mp_participant);}

bool vehicle_trajectory_waypointPublisher::init()
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
    Wparam.topic.topicName = "vehicle_trajectory_waypointPubSubTopic";

    mp_publisher = Domain::createPublisher(mp_participant,Wparam,static_cast<PublisherListener*>(&m_listener));

    if(mp_publisher == nullptr)
    {
        return false;
    }

    std::cout << "Publisher created, waiting for Subscribers." << std::endl;
    return true;
}

void vehicle_trajectory_waypointPublisher::PubListener::onPublicationMatched(Publisher* pub,MatchingInfo& info)
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

void vehicle_trajectory_waypointPublisher::run(const px4_msgs::VehicleTrajectoryWaypoint& input)
{
    vehicle_trajectory_waypoint st;

    st.timestamp_(input.timestamp);        
    st.type_(input.type);    

    std::array<trajectory_waypoint, 5> waypoints;
    for (int i = 0; i < 5; i++)
    {
        px4_msgs::TrajectoryWaypoint twp_msgs = input.waypoints[i];
        
        trajectory_waypoint twp;
        twp.timestamp_(twp_msgs.timestamp);

        std::array<float, 3> position = { twp_msgs.position[0], twp_msgs.position[1], twp_msgs.position[2] };
        twp.position(position);

        std::array<float, 3> velocity = { twp_msgs.velocity[0], twp_msgs.velocity[1], twp_msgs.velocity[2] };
        twp.velocity(velocity);

        std::array<float, 3> acceleration = { twp_msgs.acceleration[0], twp_msgs.acceleration[1], twp_msgs.acceleration[2] };
        twp.acceleration(acceleration);

        twp.yaw_(twp_msgs.yaw);
        twp.yaw_speed_(twp_msgs.yaw_speed);

        twp.type_(twp_msgs.type);
        twp.point_valid_(twp_msgs.point_valid);

        waypoints[i] = twp;
    }

    st.waypoints(waypoints);

    mp_publisher->write(&st);
}
