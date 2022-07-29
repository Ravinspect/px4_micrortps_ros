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
 * @file satellite_infoSubscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "satellite_infoSubscriber.h"

#include "ros/ros.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

satellite_infoSubscriber::satellite_infoSubscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

satellite_infoSubscriber::~satellite_infoSubscriber() {	Domain::removeParticipant(mp_participant);}

bool satellite_infoSubscriber::init(const ros::NodeHandle& private_nh)
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
    Rparam.topic.topicName = "satellite_infoPubSubTopic";
    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
    {
        return false;
    }
    m_listener.nh_ = private_nh;

    return true;
}

void satellite_infoSubscriber::SubListener::onSubscriptionMatched(Subscriber* sub,MatchingInfo& info)
{
    (void)sub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber matched" << std::endl;
        satellite_info_publisher_ = nh_.advertise<px4_msgs::SatelliteInfo>("/px4_micrortps_ros/satellite_info/out", 10);

    }
    else
    {
        n_matched--;
        std::cout << "Subscriber unmatched" << std::endl;
    }
}

void satellite_infoSubscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    // Take data
    satellite_info st;

    if(sub->takeNextData(&st, &m_info))
    {
        if(m_info.sampleKind == ALIVE)
        {
            px4_msgs::SatelliteInfoPtr msg_ptr(new px4_msgs::SatelliteInfo);
            convertUORBToPX4Message(st, msg_ptr); 

            satellite_info_publisher_.publish(msg_ptr);
        }
    }
}

void satellite_infoSubscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}

void satellite_infoSubscriber::SubListener::convertUORBToPX4Message(const satellite_info& uorb_msg, px4_msgs::SatelliteInfoPtr& px4_msg)
{

    px4_msg->timestamp = uorb_msg.timestamp_();
    px4_msg->count = uorb_msg.count_();

    ROS_INFO("Satellite Info: %d", px4_msg->count);

    for (int i = 0; i < uorb_msg.svid().size(); i++)
    {
        px4_msg->svid[i] = uorb_msg.svid()[i];
    }
    
    for (int i = 0; i < uorb_msg.used().size(); i++)
    {
        px4_msg->used[i] = uorb_msg.used()[i];
    }

    for (int i = 0; i < uorb_msg.elevation().size(); i++)
    {
        px4_msg->elevation[i] = uorb_msg.elevation()[i];
    }

    for (int i = 0; i < uorb_msg.azimuth().size(); i++)
    {
        px4_msg->azimuth[i] = uorb_msg.azimuth()[i];
    }

    for (int i = 0; i < uorb_msg.snr().size(); i++)
    {
        px4_msg->snr[i] = uorb_msg.snr()[i];
    }

    for (int i = 0; i < uorb_msg.prn().size(); i++)
    {
        px4_msg->prn[i] = uorb_msg.prn()[i];
    }
}