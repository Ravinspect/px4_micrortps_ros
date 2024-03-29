/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RtpsTopics.h"

bool RtpsTopics::init(std::condition_variable *t_send_queue_cv, std::mutex *t_send_queue_mutex,
		      std::queue<uint8_t> *t_send_queue, const std::string &ns)
{
	// Initialise subscribers
	std::cout << "\033[0;36m---   Subscribers   ---\033[0m" << std::endl;

	if (_debug_array_sub.init(12, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- debug_array subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting debug_array subscriber" << std::endl;
		return false;
	}


	if (_debug_key_value_sub.init(13, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- debug_key_value subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting debug_key_value subscriber" << std::endl;
		return false;
	}


	if (_debug_value_sub.init(14, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- debug_value subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting debug_value subscriber" << std::endl;
		return false;
	}


	if (_debug_vect_sub.init(15, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- debug_vect subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting debug_vect subscriber" << std::endl;
		return false;
	}


	if (_offboard_control_mode_sub.init(44, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- offboard_control_mode subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting offboard_control_mode subscriber" << std::endl;
		return false;
	}


	if (_optical_flow_sub.init(45, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- optical_flow subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting optical_flow subscriber" << std::endl;
		return false;
	}


	if (_position_setpoint_sub.init(50, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- position_setpoint subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting position_setpoint subscriber" << std::endl;
		return false;
	}


	if (_position_setpoint_triplet_sub.init(51, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- position_setpoint_triplet subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting position_setpoint_triplet subscriber" << std::endl;
		return false;
	}


	if (_telemetry_status_sub.init(76, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- telemetry_status subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting telemetry_status subscriber" << std::endl;
		return false;
	}


	if (_timesync_sub.init(78, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- timesync subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting timesync subscriber" << std::endl;
		return false;
	}


	if (_vehicle_command_sub.init(89, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- vehicle_command subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting vehicle_command subscriber" << std::endl;
		return false;
	}


	if (_vehicle_local_position_setpoint_sub.init(97, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- vehicle_local_position_setpoint subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting vehicle_local_position_setpoint subscriber" << std::endl;
		return false;
	}


	if (_vehicle_trajectory_waypoint_sub.init(104, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- vehicle_trajectory_waypoint subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting vehicle_trajectory_waypoint subscriber" << std::endl;
		return false;
	}


	if (_onboard_computer_status_sub.init(116, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- onboard_computer_status subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting onboard_computer_status subscriber" << std::endl;
		return false;
	}


	if (_trajectory_bezier_sub.init(126, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- trajectory_bezier subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting trajectory_bezier subscriber" << std::endl;
		return false;
	}


	if (_vehicle_trajectory_bezier_sub.init(127, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- vehicle_trajectory_bezier subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting vehicle_trajectory_bezier subscriber" << std::endl;
		return false;
	}


	if (_vehicle_mocap_odometry_sub.init(191, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- vehicle_mocap_odometry subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting vehicle_mocap_odometry subscriber" << std::endl;
		return false;
	}


	if (_vehicle_visual_odometry_sub.init(192, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- vehicle_visual_odometry subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting vehicle_visual_odometry subscriber" << std::endl;
		return false;
	}


	if (_trajectory_setpoint_sub.init(196, t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- trajectory_setpoint subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting trajectory_setpoint subscriber" << std::endl;
		return false;
	}

	std::cout << "\033[0;36m-----------------------\033[0m" << std::endl << std::endl;
	// Initialise publishers
	std::cout << "\033[0;36m----   Publishers  ----\033[0m" << std::endl;

	if (_battery_status_pub.init(ns)) {
		std::cout << "- battery_status publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting battery_status publisher" << std::endl;
		return false;
	}


	if (_distance_sensor_pub.init(ns)) {
		std::cout << "- distance_sensor publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting distance_sensor publisher" << std::endl;
		return false;
	}


	if (_input_rc_pub.init(ns)) {
		std::cout << "- input_rc publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting input_rc publisher" << std::endl;
		return false;
	}


	if (_satellite_info_pub.init(ns)) {
		std::cout << "- satellite_info publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting satellite_info publisher" << std::endl;
		return false;
	}


	if (_sensor_combined_pub.init(ns)) {
		std::cout << "- sensor_combined publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting sensor_combined publisher" << std::endl;
		return false;
	}


	if (_timesync_pub.init(ns)) {
		std::cout << "- timesync publisher started" << std::endl;
		_timesync->start(&_timesync_pub);

	} else {
		std::cerr << "ERROR starting timesync publisher" << std::endl;
		return false;
	}


	if (_trajectory_waypoint_pub.init(ns)) {
		std::cout << "- trajectory_waypoint publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting trajectory_waypoint publisher" << std::endl;
		return false;
	}


	if (_vehicle_attitude_pub.init(ns)) {
		std::cout << "- vehicle_attitude publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting vehicle_attitude publisher" << std::endl;
		return false;
	}


	if (_vehicle_control_mode_pub.init(ns)) {
		std::cout << "- vehicle_control_mode publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting vehicle_control_mode publisher" << std::endl;
		return false;
	}


	if (_vehicle_local_position_pub.init(ns)) {
		std::cout << "- vehicle_local_position publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting vehicle_local_position publisher" << std::endl;
		return false;
	}


	if (_vehicle_odometry_pub.init(ns)) {
		std::cout << "- vehicle_odometry publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting vehicle_odometry publisher" << std::endl;
		return false;
	}


	if (_vehicle_status_pub.init(ns)) {
		std::cout << "- vehicle_status publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting vehicle_status publisher" << std::endl;
		return false;
	}


	if (_collision_constraints_pub.init(ns)) {
		std::cout << "- collision_constraints publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting collision_constraints publisher" << std::endl;
		return false;
	}


	if (_vehicle_angular_velocity_pub.init(ns)) {
		std::cout << "- vehicle_angular_velocity publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting vehicle_angular_velocity publisher" << std::endl;
		return false;
	}


	if (_vehicle_trajectory_waypoint_desired_pub.init(ns)) {
		std::cout << "- vehicle_trajectory_waypoint_desired publisher started" << std::endl;

	} else {
		std::cerr << "ERROR starting vehicle_trajectory_waypoint_desired publisher" << std::endl;
		return false;
	}

	std::cout << "\033[0;36m-----------------------\033[0m" << std::endl;
	return true;
}

void RtpsTopics::publish(const uint8_t topic_ID, char data_buffer[], size_t len)
{
	switch (topic_ID) {

	case 6: { // battery_status
		battery_status_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_battery_status_pub.publish(&st);
	}
	break;

	case 17: { // distance_sensor
		distance_sensor_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_distance_sensor_pub.publish(&st);
	}
	break;

	case 30: { // input_rc
		input_rc_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_input_rc_pub.publish(&st);
	}
	break;

	case 61: { // satellite_info
		satellite_info_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_satellite_info_pub.publish(&st);
	}
	break;

	case 65: { // sensor_combined
		sensor_combined_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_sensor_combined_pub.publish(&st);
	}
	break;

	case 78: { // timesync
		timesync_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
		_timesync->processTimesyncMsg(&st, &_timesync_pub);

		if (getMsgSysID(&st) == 1) {
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_timesync_pub.publish(&st);
		}

	}
	break;

	case 79: { // trajectory_waypoint
		trajectory_waypoint_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_trajectory_waypoint_pub.publish(&st);
	}
	break;

	case 87: { // vehicle_attitude
		vehicle_attitude_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_vehicle_attitude_pub.publish(&st);
	}
	break;

	case 92: { // vehicle_control_mode
		vehicle_control_mode_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_vehicle_control_mode_pub.publish(&st);
	}
	break;

	case 96: { // vehicle_local_position
		vehicle_local_position_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_vehicle_local_position_pub.publish(&st);
	}
	break;

	case 99: { // vehicle_odometry
		vehicle_odometry_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_vehicle_odometry_pub.publish(&st);
	}
	break;

	case 102: { // vehicle_status
		vehicle_status_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_vehicle_status_pub.publish(&st);
	}
	break;

	case 107: { // collision_constraints
		collision_constraints_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_collision_constraints_pub.publish(&st);
	}
	break;

	case 112: { // vehicle_angular_velocity
		vehicle_angular_velocity_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_vehicle_angular_velocity_pub.publish(&st);
	}
	break;

	case 193: { // vehicle_trajectory_waypoint_desired
		vehicle_trajectory_waypoint_desired_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
			// apply timestamp offset
			uint64_t timestamp = getMsgTimestamp(&st);
			uint64_t timestamp_sample = getMsgTimestampSample(&st);
			_timesync->subtractOffset(timestamp);
			setMsgTimestamp(&st, timestamp);
			_timesync->subtractOffset(timestamp_sample);
			setMsgTimestampSample(&st, timestamp_sample);
			_vehicle_trajectory_waypoint_desired_pub.publish(&st);
	}
	break;

	default:
		printf("\033[1;33m[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to publish. Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'\033[0m\n",
		       topic_ID, topic_ID);
		break;
	}
}

bool RtpsTopics::getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr)
{
	bool ret = false;

	switch (topic_ID) {

	case 12: // debug_array
		if (_debug_array_sub.hasMsg()) {
			debug_array_msg_t msg = _debug_array_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_debug_array_sub.unlockMsg();
		}

		break;

	case 13: // debug_key_value
		if (_debug_key_value_sub.hasMsg()) {
			debug_key_value_msg_t msg = _debug_key_value_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_debug_key_value_sub.unlockMsg();
		}

		break;

	case 14: // debug_value
		if (_debug_value_sub.hasMsg()) {
			debug_value_msg_t msg = _debug_value_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_debug_value_sub.unlockMsg();
		}

		break;

	case 15: // debug_vect
		if (_debug_vect_sub.hasMsg()) {
			debug_vect_msg_t msg = _debug_vect_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_debug_vect_sub.unlockMsg();
		}

		break;

	case 44: // offboard_control_mode
		if (_offboard_control_mode_sub.hasMsg()) {
			offboard_control_mode_msg_t msg = _offboard_control_mode_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_offboard_control_mode_sub.unlockMsg();
		}

		break;

	case 45: // optical_flow
		if (_optical_flow_sub.hasMsg()) {
			optical_flow_msg_t msg = _optical_flow_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_optical_flow_sub.unlockMsg();
		}

		break;

	case 50: // position_setpoint
		if (_position_setpoint_sub.hasMsg()) {
			position_setpoint_msg_t msg = _position_setpoint_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_position_setpoint_sub.unlockMsg();
		}

		break;

	case 51: // position_setpoint_triplet
		if (_position_setpoint_triplet_sub.hasMsg()) {
			position_setpoint_triplet_msg_t msg = _position_setpoint_triplet_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_position_setpoint_triplet_sub.unlockMsg();
		}

		break;

	case 76: // telemetry_status
		if (_telemetry_status_sub.hasMsg()) {
			telemetry_status_msg_t msg = _telemetry_status_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_telemetry_status_sub.unlockMsg();
		}

		break;

	case 78: // timesync
		if (_timesync_sub.hasMsg()) {
			timesync_msg_t msg = _timesync_sub.getMsg();

			if (getMsgSysID(&msg) == 0) {
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			}

			_timesync_sub.unlockMsg();
		}

		break;

	case 89: // vehicle_command
		if (_vehicle_command_sub.hasMsg()) {
			vehicle_command_msg_t msg = _vehicle_command_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_vehicle_command_sub.unlockMsg();
		}

		break;

	case 97: // vehicle_local_position_setpoint
		if (_vehicle_local_position_setpoint_sub.hasMsg()) {
			vehicle_local_position_setpoint_msg_t msg = _vehicle_local_position_setpoint_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_vehicle_local_position_setpoint_sub.unlockMsg();
		}

		break;

	case 104: // vehicle_trajectory_waypoint
		if (_vehicle_trajectory_waypoint_sub.hasMsg()) {
			vehicle_trajectory_waypoint_msg_t msg = _vehicle_trajectory_waypoint_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_vehicle_trajectory_waypoint_sub.unlockMsg();
		}

		break;

	case 116: // onboard_computer_status
		if (_onboard_computer_status_sub.hasMsg()) {
			onboard_computer_status_msg_t msg = _onboard_computer_status_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_onboard_computer_status_sub.unlockMsg();
		}

		break;

	case 126: // trajectory_bezier
		if (_trajectory_bezier_sub.hasMsg()) {
			trajectory_bezier_msg_t msg = _trajectory_bezier_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_trajectory_bezier_sub.unlockMsg();
		}

		break;

	case 127: // vehicle_trajectory_bezier
		if (_vehicle_trajectory_bezier_sub.hasMsg()) {
			vehicle_trajectory_bezier_msg_t msg = _vehicle_trajectory_bezier_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_vehicle_trajectory_bezier_sub.unlockMsg();
		}

		break;

	case 191: // vehicle_mocap_odometry
		if (_vehicle_mocap_odometry_sub.hasMsg()) {
			vehicle_mocap_odometry_msg_t msg = _vehicle_mocap_odometry_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_vehicle_mocap_odometry_sub.unlockMsg();
		}

		break;

	case 192: // vehicle_visual_odometry
		if (_vehicle_visual_odometry_sub.hasMsg()) {
			vehicle_visual_odometry_msg_t msg = _vehicle_visual_odometry_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_vehicle_visual_odometry_sub.unlockMsg();
		}

		break;

	case 196: // trajectory_setpoint
		if (_trajectory_setpoint_sub.hasMsg()) {
			trajectory_setpoint_msg_t msg = _trajectory_setpoint_sub.getMsg();
				// apply timestamps offset
				uint64_t timestamp = getMsgTimestamp(&msg);
				uint64_t timestamp_sample = getMsgTimestampSample(&msg);
				_timesync->addOffset(timestamp);
				setMsgTimestamp(&msg, timestamp);
				_timesync->addOffset(timestamp_sample);
				setMsgTimestampSample(&msg, timestamp_sample);
				msg.serialize(scdr);
				ret = true;
			_trajectory_setpoint_sub.unlockMsg();
		}

		break;

	default:
		printf("\033[1;33m[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to getMsg. Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'\033[0m\n",
		       topic_ID, topic_ID);
		break;
	}

	return ret;
}
