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
 * @file onboard_computer_statusPublisher.cpp
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

#include "onboard_computer_statusPublisher.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

onboard_computer_statusPublisher::onboard_computer_statusPublisher() : mp_participant(nullptr), mp_publisher(nullptr) {}

onboard_computer_statusPublisher::~onboard_computer_statusPublisher() {	Domain::removeParticipant(mp_participant);}

bool onboard_computer_statusPublisher::init()
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
    Wparam.topic.topicName = "onboard_computer_statusPubSubTopic";

    mp_publisher = Domain::createPublisher(mp_participant,Wparam,static_cast<PublisherListener*>(&m_listener));

    if(mp_publisher == nullptr)
    {
        return false;
    }

    std::cout << "Publisher created, waiting for Subscribers." << std::endl;
    return true;
}

void onboard_computer_statusPublisher::PubListener::onPublicationMatched(Publisher* pub,MatchingInfo& info)
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

void onboard_computer_statusPublisher::run(const px4_msgs::OnboardComputerStatus& input)
{
    onboard_computer_status st;

    st.timestamp_(input.timestamp);     
    st.uptime_(input.uptime);

    st.type_(input.type);
            
    std::array<uint8_t, 8> cpu_cores = { input.cpu_cores[0], input.cpu_cores[1], input.cpu_cores[2], input.cpu_cores[3], input.cpu_cores[4], input.cpu_cores[5], input.cpu_cores[6], input.cpu_cores[7] };
    st.cpu_cores(cpu_cores);

    std::array<uint8_t, 10> cpu_combined = { input.cpu_combined[0], input.cpu_combined[1], input.cpu_combined[2], input.cpu_combined[3], input.cpu_combined[4], input.cpu_combined[5], input.cpu_combined[6], input.cpu_combined[7], input.cpu_combined[8], input.cpu_combined[9] };
    st.cpu_combined(cpu_combined);
    
    std::array<uint8_t, 4> gpu_cores = { input.gpu_cores[0], input.gpu_cores[1], input.gpu_cores[2], input.gpu_cores[3] };
    st.gpu_cores(gpu_cores);

    std::array<uint8_t, 10> gpu_combined = { input.gpu_combined[0], input.gpu_combined[1], input.gpu_combined[2], input.gpu_combined[3], input.gpu_combined[4], input.gpu_combined[5], input.gpu_combined[6], input.gpu_combined[7], input.gpu_combined[8], input.gpu_combined[9] };
    st.gpu_combined(gpu_combined);

    st.temperature_board_(input.temperature_board);

    std::array<uint8_t, 8> temperature_core = { input.temperature_core[0], input.temperature_core[1], input.temperature_core[2], input.temperature_core[3], input.temperature_core[4], input.temperature_core[5], input.temperature_core[6], input.temperature_core[7] };
    st.temperature_core(temperature_core);

    std::array<int16_t, 4> fan_speed = { input.fan_speed[0], input.fan_speed[1], input.fan_speed[2], input.fan_speed[3] };
    st.fan_speed(fan_speed);

    st.ram_usage_(input.ram_usage);
    st.ram_total_(input.ram_total);

    std::array<uint32_t, 4> storage_type = { input.storage_type[0], input.storage_type[1], input.storage_type[2], input.storage_type[3] };
    st.storage_type(storage_type);

    std::array<uint32_t, 4> storage_usage = { input.storage_usage[0], input.storage_usage[1], input.storage_usage[2], input.storage_usage[3] };
    st.storage_usage(storage_usage);
    
    std::array<uint32_t, 4> storage_total = { input.storage_total[0], input.storage_total[1], input.storage_total[2], input.storage_total[3] };
    st.storage_total(storage_total);
    
    std::array<uint32_t, 6> link_type = { input.link_type[0], input.link_type[1], input.link_type[2], input.link_type[3], input.link_type[4], input.link_type[5] };
    st.link_type(link_type);
    
    std::array<uint32_t, 6> link_tx_rate = { input.link_tx_rate[0], input.link_tx_rate[1], input.link_tx_rate[2], input.link_tx_rate[3], input.link_tx_rate[4], input.link_tx_rate[5] };
    st.link_tx_rate(link_tx_rate);

    std::array<uint32_t, 6> link_rx_rate = { input.link_rx_rate[0], input.link_rx_rate[1], input.link_rx_rate[2], input.link_rx_rate[3], input.link_rx_rate[4], input.link_rx_rate[5] };
    st.link_rx_rate(link_rx_rate);
    
    std::array<uint32_t, 6> link_tx_max = { input.link_tx_max[0], input.link_tx_max[1], input.link_tx_max[2], input.link_tx_max[3], input.link_tx_max[4], input.link_tx_max[5] };
    st.link_tx_max(link_tx_max);

    std::array<uint32_t, 6> link_rx_max = { input.link_rx_max[0], input.link_rx_max[1], input.link_rx_max[2], input.link_rx_max[3], input.link_rx_max[4], input.link_rx_max[5] };
    st.link_rx_max(link_rx_max);

    mp_publisher->write(&st);
}
