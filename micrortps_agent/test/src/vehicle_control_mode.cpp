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
 * @file vehicle_control_mode.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "vehicle_control_mode.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

vehicle_control_mode::vehicle_control_mode()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1060b431
    m_timestamp_ = 0;
    // m_flag_armed_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@491cc5c9
    m_flag_armed_ = false;
    // m_flag_external_manual_override_ok_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@74ad1f1f
    m_flag_external_manual_override_ok_ = false;
    // m_flag_multicopter_position_control_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6a1aab78
    m_flag_multicopter_position_control_enabled_ = false;
    // m_flag_control_manual_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@462d5aee
    m_flag_control_manual_enabled_ = false;
    // m_flag_control_auto_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@69b0fd6f
    m_flag_control_auto_enabled_ = false;
    // m_flag_control_offboard_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@757942a1
    m_flag_control_offboard_enabled_ = false;
    // m_flag_control_rates_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4a87761d
    m_flag_control_rates_enabled_ = false;
    // m_flag_control_attitude_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@66d1af89
    m_flag_control_attitude_enabled_ = false;
    // m_flag_control_acceleration_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@8646db9
    m_flag_control_acceleration_enabled_ = false;
    // m_flag_control_velocity_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@37374a5e
    m_flag_control_velocity_enabled_ = false;
    // m_flag_control_position_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4671e53b
    m_flag_control_position_enabled_ = false;
    // m_flag_control_altitude_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2db7a79b
    m_flag_control_altitude_enabled_ = false;
    // m_flag_control_climb_rate_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6950e31
    m_flag_control_climb_rate_enabled_ = false;
    // m_flag_control_termination_enabled_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@b7dd107
    m_flag_control_termination_enabled_ = false;

}

vehicle_control_mode::~vehicle_control_mode()
{















}

vehicle_control_mode::vehicle_control_mode(const vehicle_control_mode &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_flag_armed_ = x.m_flag_armed_;
    m_flag_external_manual_override_ok_ = x.m_flag_external_manual_override_ok_;
    m_flag_multicopter_position_control_enabled_ = x.m_flag_multicopter_position_control_enabled_;
    m_flag_control_manual_enabled_ = x.m_flag_control_manual_enabled_;
    m_flag_control_auto_enabled_ = x.m_flag_control_auto_enabled_;
    m_flag_control_offboard_enabled_ = x.m_flag_control_offboard_enabled_;
    m_flag_control_rates_enabled_ = x.m_flag_control_rates_enabled_;
    m_flag_control_attitude_enabled_ = x.m_flag_control_attitude_enabled_;
    m_flag_control_acceleration_enabled_ = x.m_flag_control_acceleration_enabled_;
    m_flag_control_velocity_enabled_ = x.m_flag_control_velocity_enabled_;
    m_flag_control_position_enabled_ = x.m_flag_control_position_enabled_;
    m_flag_control_altitude_enabled_ = x.m_flag_control_altitude_enabled_;
    m_flag_control_climb_rate_enabled_ = x.m_flag_control_climb_rate_enabled_;
    m_flag_control_termination_enabled_ = x.m_flag_control_termination_enabled_;
}

vehicle_control_mode::vehicle_control_mode(vehicle_control_mode &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_flag_armed_ = x.m_flag_armed_;
    m_flag_external_manual_override_ok_ = x.m_flag_external_manual_override_ok_;
    m_flag_multicopter_position_control_enabled_ = x.m_flag_multicopter_position_control_enabled_;
    m_flag_control_manual_enabled_ = x.m_flag_control_manual_enabled_;
    m_flag_control_auto_enabled_ = x.m_flag_control_auto_enabled_;
    m_flag_control_offboard_enabled_ = x.m_flag_control_offboard_enabled_;
    m_flag_control_rates_enabled_ = x.m_flag_control_rates_enabled_;
    m_flag_control_attitude_enabled_ = x.m_flag_control_attitude_enabled_;
    m_flag_control_acceleration_enabled_ = x.m_flag_control_acceleration_enabled_;
    m_flag_control_velocity_enabled_ = x.m_flag_control_velocity_enabled_;
    m_flag_control_position_enabled_ = x.m_flag_control_position_enabled_;
    m_flag_control_altitude_enabled_ = x.m_flag_control_altitude_enabled_;
    m_flag_control_climb_rate_enabled_ = x.m_flag_control_climb_rate_enabled_;
    m_flag_control_termination_enabled_ = x.m_flag_control_termination_enabled_;
}

vehicle_control_mode& vehicle_control_mode::operator=(const vehicle_control_mode &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_flag_armed_ = x.m_flag_armed_;
    m_flag_external_manual_override_ok_ = x.m_flag_external_manual_override_ok_;
    m_flag_multicopter_position_control_enabled_ = x.m_flag_multicopter_position_control_enabled_;
    m_flag_control_manual_enabled_ = x.m_flag_control_manual_enabled_;
    m_flag_control_auto_enabled_ = x.m_flag_control_auto_enabled_;
    m_flag_control_offboard_enabled_ = x.m_flag_control_offboard_enabled_;
    m_flag_control_rates_enabled_ = x.m_flag_control_rates_enabled_;
    m_flag_control_attitude_enabled_ = x.m_flag_control_attitude_enabled_;
    m_flag_control_acceleration_enabled_ = x.m_flag_control_acceleration_enabled_;
    m_flag_control_velocity_enabled_ = x.m_flag_control_velocity_enabled_;
    m_flag_control_position_enabled_ = x.m_flag_control_position_enabled_;
    m_flag_control_altitude_enabled_ = x.m_flag_control_altitude_enabled_;
    m_flag_control_climb_rate_enabled_ = x.m_flag_control_climb_rate_enabled_;
    m_flag_control_termination_enabled_ = x.m_flag_control_termination_enabled_;

    return *this;
}

vehicle_control_mode& vehicle_control_mode::operator=(vehicle_control_mode &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_flag_armed_ = x.m_flag_armed_;
    m_flag_external_manual_override_ok_ = x.m_flag_external_manual_override_ok_;
    m_flag_multicopter_position_control_enabled_ = x.m_flag_multicopter_position_control_enabled_;
    m_flag_control_manual_enabled_ = x.m_flag_control_manual_enabled_;
    m_flag_control_auto_enabled_ = x.m_flag_control_auto_enabled_;
    m_flag_control_offboard_enabled_ = x.m_flag_control_offboard_enabled_;
    m_flag_control_rates_enabled_ = x.m_flag_control_rates_enabled_;
    m_flag_control_attitude_enabled_ = x.m_flag_control_attitude_enabled_;
    m_flag_control_acceleration_enabled_ = x.m_flag_control_acceleration_enabled_;
    m_flag_control_velocity_enabled_ = x.m_flag_control_velocity_enabled_;
    m_flag_control_position_enabled_ = x.m_flag_control_position_enabled_;
    m_flag_control_altitude_enabled_ = x.m_flag_control_altitude_enabled_;
    m_flag_control_climb_rate_enabled_ = x.m_flag_control_climb_rate_enabled_;
    m_flag_control_termination_enabled_ = x.m_flag_control_termination_enabled_;

    return *this;
}

size_t vehicle_control_mode::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

size_t vehicle_control_mode::getCdrSerializedSize(const vehicle_control_mode& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

void vehicle_control_mode::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_flag_armed_;
    scdr << m_flag_external_manual_override_ok_;
    scdr << m_flag_multicopter_position_control_enabled_;
    scdr << m_flag_control_manual_enabled_;
    scdr << m_flag_control_auto_enabled_;
    scdr << m_flag_control_offboard_enabled_;
    scdr << m_flag_control_rates_enabled_;
    scdr << m_flag_control_attitude_enabled_;
    scdr << m_flag_control_acceleration_enabled_;
    scdr << m_flag_control_velocity_enabled_;
    scdr << m_flag_control_position_enabled_;
    scdr << m_flag_control_altitude_enabled_;
    scdr << m_flag_control_climb_rate_enabled_;
    scdr << m_flag_control_termination_enabled_;
}

void vehicle_control_mode::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_flag_armed_;
    dcdr >> m_flag_external_manual_override_ok_;
    dcdr >> m_flag_multicopter_position_control_enabled_;
    dcdr >> m_flag_control_manual_enabled_;
    dcdr >> m_flag_control_auto_enabled_;
    dcdr >> m_flag_control_offboard_enabled_;
    dcdr >> m_flag_control_rates_enabled_;
    dcdr >> m_flag_control_attitude_enabled_;
    dcdr >> m_flag_control_acceleration_enabled_;
    dcdr >> m_flag_control_velocity_enabled_;
    dcdr >> m_flag_control_position_enabled_;
    dcdr >> m_flag_control_altitude_enabled_;
    dcdr >> m_flag_control_climb_rate_enabled_;
    dcdr >> m_flag_control_termination_enabled_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void vehicle_control_mode::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t vehicle_control_mode::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& vehicle_control_mode::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function sets a value in member flag_armed_
 * @param _flag_armed_ New value for member flag_armed_
 */
void vehicle_control_mode::flag_armed_(bool _flag_armed_)
{
m_flag_armed_ = _flag_armed_;
}

/*!
 * @brief This function returns the value of member flag_armed_
 * @return Value of member flag_armed_
 */
bool vehicle_control_mode::flag_armed_() const
{
    return m_flag_armed_;
}

/*!
 * @brief This function returns a reference to member flag_armed_
 * @return Reference to member flag_armed_
 */
bool& vehicle_control_mode::flag_armed_()
{
    return m_flag_armed_;
}

/*!
 * @brief This function sets a value in member flag_external_manual_override_ok_
 * @param _flag_external_manual_override_ok_ New value for member flag_external_manual_override_ok_
 */
void vehicle_control_mode::flag_external_manual_override_ok_(bool _flag_external_manual_override_ok_)
{
m_flag_external_manual_override_ok_ = _flag_external_manual_override_ok_;
}

/*!
 * @brief This function returns the value of member flag_external_manual_override_ok_
 * @return Value of member flag_external_manual_override_ok_
 */
bool vehicle_control_mode::flag_external_manual_override_ok_() const
{
    return m_flag_external_manual_override_ok_;
}

/*!
 * @brief This function returns a reference to member flag_external_manual_override_ok_
 * @return Reference to member flag_external_manual_override_ok_
 */
bool& vehicle_control_mode::flag_external_manual_override_ok_()
{
    return m_flag_external_manual_override_ok_;
}

/*!
 * @brief This function sets a value in member flag_multicopter_position_control_enabled_
 * @param _flag_multicopter_position_control_enabled_ New value for member flag_multicopter_position_control_enabled_
 */
void vehicle_control_mode::flag_multicopter_position_control_enabled_(bool _flag_multicopter_position_control_enabled_)
{
m_flag_multicopter_position_control_enabled_ = _flag_multicopter_position_control_enabled_;
}

/*!
 * @brief This function returns the value of member flag_multicopter_position_control_enabled_
 * @return Value of member flag_multicopter_position_control_enabled_
 */
bool vehicle_control_mode::flag_multicopter_position_control_enabled_() const
{
    return m_flag_multicopter_position_control_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_multicopter_position_control_enabled_
 * @return Reference to member flag_multicopter_position_control_enabled_
 */
bool& vehicle_control_mode::flag_multicopter_position_control_enabled_()
{
    return m_flag_multicopter_position_control_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_manual_enabled_
 * @param _flag_control_manual_enabled_ New value for member flag_control_manual_enabled_
 */
void vehicle_control_mode::flag_control_manual_enabled_(bool _flag_control_manual_enabled_)
{
m_flag_control_manual_enabled_ = _flag_control_manual_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_manual_enabled_
 * @return Value of member flag_control_manual_enabled_
 */
bool vehicle_control_mode::flag_control_manual_enabled_() const
{
    return m_flag_control_manual_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_manual_enabled_
 * @return Reference to member flag_control_manual_enabled_
 */
bool& vehicle_control_mode::flag_control_manual_enabled_()
{
    return m_flag_control_manual_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_auto_enabled_
 * @param _flag_control_auto_enabled_ New value for member flag_control_auto_enabled_
 */
void vehicle_control_mode::flag_control_auto_enabled_(bool _flag_control_auto_enabled_)
{
m_flag_control_auto_enabled_ = _flag_control_auto_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_auto_enabled_
 * @return Value of member flag_control_auto_enabled_
 */
bool vehicle_control_mode::flag_control_auto_enabled_() const
{
    return m_flag_control_auto_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_auto_enabled_
 * @return Reference to member flag_control_auto_enabled_
 */
bool& vehicle_control_mode::flag_control_auto_enabled_()
{
    return m_flag_control_auto_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_offboard_enabled_
 * @param _flag_control_offboard_enabled_ New value for member flag_control_offboard_enabled_
 */
void vehicle_control_mode::flag_control_offboard_enabled_(bool _flag_control_offboard_enabled_)
{
m_flag_control_offboard_enabled_ = _flag_control_offboard_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_offboard_enabled_
 * @return Value of member flag_control_offboard_enabled_
 */
bool vehicle_control_mode::flag_control_offboard_enabled_() const
{
    return m_flag_control_offboard_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_offboard_enabled_
 * @return Reference to member flag_control_offboard_enabled_
 */
bool& vehicle_control_mode::flag_control_offboard_enabled_()
{
    return m_flag_control_offboard_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_rates_enabled_
 * @param _flag_control_rates_enabled_ New value for member flag_control_rates_enabled_
 */
void vehicle_control_mode::flag_control_rates_enabled_(bool _flag_control_rates_enabled_)
{
m_flag_control_rates_enabled_ = _flag_control_rates_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_rates_enabled_
 * @return Value of member flag_control_rates_enabled_
 */
bool vehicle_control_mode::flag_control_rates_enabled_() const
{
    return m_flag_control_rates_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_rates_enabled_
 * @return Reference to member flag_control_rates_enabled_
 */
bool& vehicle_control_mode::flag_control_rates_enabled_()
{
    return m_flag_control_rates_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_attitude_enabled_
 * @param _flag_control_attitude_enabled_ New value for member flag_control_attitude_enabled_
 */
void vehicle_control_mode::flag_control_attitude_enabled_(bool _flag_control_attitude_enabled_)
{
m_flag_control_attitude_enabled_ = _flag_control_attitude_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_attitude_enabled_
 * @return Value of member flag_control_attitude_enabled_
 */
bool vehicle_control_mode::flag_control_attitude_enabled_() const
{
    return m_flag_control_attitude_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_attitude_enabled_
 * @return Reference to member flag_control_attitude_enabled_
 */
bool& vehicle_control_mode::flag_control_attitude_enabled_()
{
    return m_flag_control_attitude_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_acceleration_enabled_
 * @param _flag_control_acceleration_enabled_ New value for member flag_control_acceleration_enabled_
 */
void vehicle_control_mode::flag_control_acceleration_enabled_(bool _flag_control_acceleration_enabled_)
{
m_flag_control_acceleration_enabled_ = _flag_control_acceleration_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_acceleration_enabled_
 * @return Value of member flag_control_acceleration_enabled_
 */
bool vehicle_control_mode::flag_control_acceleration_enabled_() const
{
    return m_flag_control_acceleration_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_acceleration_enabled_
 * @return Reference to member flag_control_acceleration_enabled_
 */
bool& vehicle_control_mode::flag_control_acceleration_enabled_()
{
    return m_flag_control_acceleration_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_velocity_enabled_
 * @param _flag_control_velocity_enabled_ New value for member flag_control_velocity_enabled_
 */
void vehicle_control_mode::flag_control_velocity_enabled_(bool _flag_control_velocity_enabled_)
{
m_flag_control_velocity_enabled_ = _flag_control_velocity_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_velocity_enabled_
 * @return Value of member flag_control_velocity_enabled_
 */
bool vehicle_control_mode::flag_control_velocity_enabled_() const
{
    return m_flag_control_velocity_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_velocity_enabled_
 * @return Reference to member flag_control_velocity_enabled_
 */
bool& vehicle_control_mode::flag_control_velocity_enabled_()
{
    return m_flag_control_velocity_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_position_enabled_
 * @param _flag_control_position_enabled_ New value for member flag_control_position_enabled_
 */
void vehicle_control_mode::flag_control_position_enabled_(bool _flag_control_position_enabled_)
{
m_flag_control_position_enabled_ = _flag_control_position_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_position_enabled_
 * @return Value of member flag_control_position_enabled_
 */
bool vehicle_control_mode::flag_control_position_enabled_() const
{
    return m_flag_control_position_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_position_enabled_
 * @return Reference to member flag_control_position_enabled_
 */
bool& vehicle_control_mode::flag_control_position_enabled_()
{
    return m_flag_control_position_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_altitude_enabled_
 * @param _flag_control_altitude_enabled_ New value for member flag_control_altitude_enabled_
 */
void vehicle_control_mode::flag_control_altitude_enabled_(bool _flag_control_altitude_enabled_)
{
m_flag_control_altitude_enabled_ = _flag_control_altitude_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_altitude_enabled_
 * @return Value of member flag_control_altitude_enabled_
 */
bool vehicle_control_mode::flag_control_altitude_enabled_() const
{
    return m_flag_control_altitude_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_altitude_enabled_
 * @return Reference to member flag_control_altitude_enabled_
 */
bool& vehicle_control_mode::flag_control_altitude_enabled_()
{
    return m_flag_control_altitude_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_climb_rate_enabled_
 * @param _flag_control_climb_rate_enabled_ New value for member flag_control_climb_rate_enabled_
 */
void vehicle_control_mode::flag_control_climb_rate_enabled_(bool _flag_control_climb_rate_enabled_)
{
m_flag_control_climb_rate_enabled_ = _flag_control_climb_rate_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_climb_rate_enabled_
 * @return Value of member flag_control_climb_rate_enabled_
 */
bool vehicle_control_mode::flag_control_climb_rate_enabled_() const
{
    return m_flag_control_climb_rate_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_climb_rate_enabled_
 * @return Reference to member flag_control_climb_rate_enabled_
 */
bool& vehicle_control_mode::flag_control_climb_rate_enabled_()
{
    return m_flag_control_climb_rate_enabled_;
}

/*!
 * @brief This function sets a value in member flag_control_termination_enabled_
 * @param _flag_control_termination_enabled_ New value for member flag_control_termination_enabled_
 */
void vehicle_control_mode::flag_control_termination_enabled_(bool _flag_control_termination_enabled_)
{
m_flag_control_termination_enabled_ = _flag_control_termination_enabled_;
}

/*!
 * @brief This function returns the value of member flag_control_termination_enabled_
 * @return Value of member flag_control_termination_enabled_
 */
bool vehicle_control_mode::flag_control_termination_enabled_() const
{
    return m_flag_control_termination_enabled_;
}

/*!
 * @brief This function returns a reference to member flag_control_termination_enabled_
 * @return Reference to member flag_control_termination_enabled_
 */
bool& vehicle_control_mode::flag_control_termination_enabled_()
{
    return m_flag_control_termination_enabled_;
}


size_t vehicle_control_mode::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;


















    return current_align;
}

bool vehicle_control_mode::isKeyDefined()
{
   return false;
}

void vehicle_control_mode::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
}
