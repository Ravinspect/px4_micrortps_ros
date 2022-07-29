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
 * @file vehicle_trajectory_waypointSubscriber.h
 * This header file contains the declaration of the subscriber functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#ifndef _VEHICLE_TRAJECTORY_WAYPOINT_SUBSCRIBER_H_
#define _VEHICLE_TRAJECTORY_WAYPOINT_SUBSCRIBER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include "vehicle_trajectory_waypointPubSubTypes.h"

class vehicle_trajectory_waypointSubscriber
{
public:
	vehicle_trajectory_waypointSubscriber();
	virtual ~vehicle_trajectory_waypointSubscriber();
	bool init();
	void run();
private:
	eprosima::fastrtps::Participant *mp_participant;
	eprosima::fastrtps::Subscriber *mp_subscriber;

	class SubListener : public eprosima::fastrtps::SubscriberListener
	{
	public:
		SubListener() : n_matched(0),n_msg(0){};
		~SubListener(){};
		void onSubscriptionMatched(eprosima::fastrtps::Subscriber* sub,eprosima::fastrtps::rtps::MatchingInfo& info);
		void onNewDataMessage(eprosima::fastrtps::Subscriber* sub);
		eprosima::fastrtps::SampleInfo_t m_info;
		int n_matched;
		int n_msg;
	} m_listener;
	vehicle_trajectory_waypointPubSubType myType;
};

#endif // _VEHICLE_TRAJECTORY_WAYPOINT_SUBSCRIBER_H_