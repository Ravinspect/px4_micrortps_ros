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
 * @file collision_constraintsSubscriber.cpp
 * This file contains the implementation of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/attributes/SubscriberAttributes.h>

#include <fastrtps/Domain.h>

#include "collision_constraintsSubscriber.h"

#include "ros/ros.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

collision_constraintsSubscriber::collision_constraintsSubscriber() : mp_participant(nullptr), mp_subscriber(nullptr) {}

collision_constraintsSubscriber::~collision_constraintsSubscriber() {	Domain::removeParticipant(mp_participant);}

bool collision_constraintsSubscriber::init(const ros::NodeHandle& private_nh)
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
    Rparam.topic.topicName = "collision_constraintsPubSubTopic";
    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam, static_cast<SubscriberListener*>(&m_listener));
    if(mp_subscriber == nullptr)
    {
        return false;
    }

    m_listener.nh_ = private_nh;

    return true;
}

void collision_constraintsSubscriber::SubListener::onSubscriptionMatched(Subscriber* sub,MatchingInfo& info)
{
    (void)sub;

    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber matched" << std::endl;
        collision_constraints_publisher_ = nh_.advertise<px4_msgs::CollisionConstraints>("/px4_micrortps_ros/collision_constraints/out", 10);
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber unmatched" << std::endl;
    }
}

void collision_constraintsSubscriber::SubListener::onNewDataMessage(Subscriber* sub)
{
    // Take data
    collision_constraints st;

    if(sub->takeNextData(&st, &m_info))
    {
        if(m_info.sampleKind == ALIVE)
        {
            px4_msgs::CollisionConstraintsPtr msg_ptr(new px4_msgs::CollisionConstraints);
            convertUORBToPX4Message(st, msg_ptr); 
            collision_constraints_publisher_.publish(msg_ptr);
        }
    }
}

void collision_constraintsSubscriber::run()
{
    std::cout << "Waiting for Data, press Enter to stop the Subscriber. "<<std::endl;
    std::cin.ignore();
    std::cout << "Shutting down the Subscriber." << std::endl;
}


void collision_constraintsSubscriber::SubListener::convertUORBToPX4Message(const collision_constraints& uorb_msg,  px4_msgs::CollisionConstraintsPtr& px4_msg_ptr)
{
    px4_msg_ptr->timestamp = uorb_msg.timestamp_();

    px4_msg_ptr->original_setpoint[0] = uorb_msg.original_setpoint()[0];
    px4_msg_ptr->original_setpoint[1] = uorb_msg.original_setpoint()[1];
    
    px4_msg_ptr->adapted_setpoint[0] = uorb_msg.adapted_setpoint()[0];
    px4_msg_ptr->adapted_setpoint[1] = uorb_msg.adapted_setpoint()[1];
}
