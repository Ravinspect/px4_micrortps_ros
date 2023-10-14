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
 * @file vehicle_trajectory_waypoint_desired.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "vehicle_trajectory_waypoint_desired.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>










vehicle_trajectory_waypoint_desired::vehicle_trajectory_waypoint_desired()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@588df31b
    m_timestamp_ = 0;
    // m_type_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@33b37288
    m_type_ = 0;
    // m_waypoints com.eprosima.idl.parser.typecode.AliasTypeCode@77a57272


}

vehicle_trajectory_waypoint_desired::~vehicle_trajectory_waypoint_desired()
{



}

vehicle_trajectory_waypoint_desired::vehicle_trajectory_waypoint_desired(const vehicle_trajectory_waypoint_desired &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_type_ = x.m_type_;
    m_waypoints = x.m_waypoints;
}

vehicle_trajectory_waypoint_desired::vehicle_trajectory_waypoint_desired(vehicle_trajectory_waypoint_desired &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_type_ = x.m_type_;
    m_waypoints = std::move(x.m_waypoints);
}

vehicle_trajectory_waypoint_desired& vehicle_trajectory_waypoint_desired::operator=(const vehicle_trajectory_waypoint_desired &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_type_ = x.m_type_;
    m_waypoints = x.m_waypoints;

    return *this;
}

vehicle_trajectory_waypoint_desired& vehicle_trajectory_waypoint_desired::operator=(vehicle_trajectory_waypoint_desired &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_type_ = x.m_type_;
    m_waypoints = std::move(x.m_waypoints);

    return *this;
}

size_t vehicle_trajectory_waypoint_desired::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    for(size_t a = 0; a < (5); ++a)
    {
        current_alignment += trajectory_waypoint::getMaxCdrSerializedSize(current_alignment);}

    return current_alignment - initial_alignment;
}

size_t vehicle_trajectory_waypoint_desired::getCdrSerializedSize(const vehicle_trajectory_waypoint_desired& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    for(size_t a = 0; a < data.waypoints().size(); ++a)
    {
            current_alignment += trajectory_waypoint::getCdrSerializedSize(data.waypoints().at(a), current_alignment);
    }

    return current_alignment - initial_alignment;
}

void vehicle_trajectory_waypoint_desired::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_type_;
    scdr << m_waypoints;

}

void vehicle_trajectory_waypoint_desired::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_type_;
    dcdr >> m_waypoints;

}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void vehicle_trajectory_waypoint_desired::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t vehicle_trajectory_waypoint_desired::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& vehicle_trajectory_waypoint_desired::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function sets a value in member type_
 * @param _type_ New value for member type_
 */
void vehicle_trajectory_waypoint_desired::type_(uint8_t _type_)
{
m_type_ = _type_;
}

/*!
 * @brief This function returns the value of member type_
 * @return Value of member type_
 */
uint8_t vehicle_trajectory_waypoint_desired::type_() const
{
    return m_type_;
}

/*!
 * @brief This function returns a reference to member type_
 * @return Reference to member type_
 */
uint8_t& vehicle_trajectory_waypoint_desired::type_()
{
    return m_type_;
}

/*!
 * @brief This function copies the value in member waypoints
 * @param _waypoints New value to be copied in member waypoints
 */
void vehicle_trajectory_waypoint_desired::waypoints(const vehicle_trajectory_waypoint_desired__trajectory_waypoint_array_5 &_waypoints)
{
m_waypoints = _waypoints;
}

/*!
 * @brief This function moves the value in member waypoints
 * @param _waypoints New value to be moved in member waypoints
 */
void vehicle_trajectory_waypoint_desired::waypoints(vehicle_trajectory_waypoint_desired__trajectory_waypoint_array_5 &&_waypoints)
{
m_waypoints = std::move(_waypoints);
}

/*!
 * @brief This function returns a constant reference to member waypoints
 * @return Constant reference to member waypoints
 */
const vehicle_trajectory_waypoint_desired__trajectory_waypoint_array_5& vehicle_trajectory_waypoint_desired::waypoints() const
{
    return m_waypoints;
}

/*!
 * @brief This function returns a reference to member waypoints
 * @return Reference to member waypoints
 */
vehicle_trajectory_waypoint_desired__trajectory_waypoint_array_5& vehicle_trajectory_waypoint_desired::waypoints()
{
    return m_waypoints;
}

size_t vehicle_trajectory_waypoint_desired::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;






    return current_align;
}

bool vehicle_trajectory_waypoint_desired::isKeyDefined()
{
   return false;
}

void vehicle_trajectory_waypoint_desired::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
}
