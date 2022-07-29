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
 * @file trajectory_bezier.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "trajectory_bezier.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>


trajectory_bezier::trajectory_bezier()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@5e955596
    m_timestamp_ = 0;
    // m_position com.eprosima.idl.parser.typecode.AliasTypeCode@50de0926
    memset(&m_position, 0, (3) * 4);
    // m_yaw_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2473b9ce
    m_yaw_ = 0.0;
    // m_delta_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@60438a68
    m_delta_ = 0.0;

}

trajectory_bezier::~trajectory_bezier()
{




}

trajectory_bezier::trajectory_bezier(const trajectory_bezier &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_position = x.m_position;
    m_yaw_ = x.m_yaw_;
    m_delta_ = x.m_delta_;
}

trajectory_bezier::trajectory_bezier(trajectory_bezier &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_position = std::move(x.m_position);
    m_yaw_ = x.m_yaw_;
    m_delta_ = x.m_delta_;
}

trajectory_bezier& trajectory_bezier::operator=(const trajectory_bezier &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_position = x.m_position;
    m_yaw_ = x.m_yaw_;
    m_delta_ = x.m_delta_;

    return *this;
}

trajectory_bezier& trajectory_bezier::operator=(trajectory_bezier &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_position = std::move(x.m_position);
    m_yaw_ = x.m_yaw_;
    m_delta_ = x.m_delta_;

    return *this;
}

size_t trajectory_bezier::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t trajectory_bezier::getCdrSerializedSize(const trajectory_bezier& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void trajectory_bezier::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_position;

    scdr << m_yaw_;
    scdr << m_delta_;
}

void trajectory_bezier::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_position;

    dcdr >> m_yaw_;
    dcdr >> m_delta_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void trajectory_bezier::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t trajectory_bezier::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& trajectory_bezier::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function copies the value in member position
 * @param _position New value to be copied in member position
 */
void trajectory_bezier::position(const trajectory_bezier__float_array_3 &_position)
{
m_position = _position;
}

/*!
 * @brief This function moves the value in member position
 * @param _position New value to be moved in member position
 */
void trajectory_bezier::position(trajectory_bezier__float_array_3 &&_position)
{
m_position = std::move(_position);
}

/*!
 * @brief This function returns a constant reference to member position
 * @return Constant reference to member position
 */
const trajectory_bezier__float_array_3& trajectory_bezier::position() const
{
    return m_position;
}

/*!
 * @brief This function returns a reference to member position
 * @return Reference to member position
 */
trajectory_bezier__float_array_3& trajectory_bezier::position()
{
    return m_position;
}
/*!
 * @brief This function sets a value in member yaw_
 * @param _yaw_ New value for member yaw_
 */
void trajectory_bezier::yaw_(float _yaw_)
{
m_yaw_ = _yaw_;
}

/*!
 * @brief This function returns the value of member yaw_
 * @return Value of member yaw_
 */
float trajectory_bezier::yaw_() const
{
    return m_yaw_;
}

/*!
 * @brief This function returns a reference to member yaw_
 * @return Reference to member yaw_
 */
float& trajectory_bezier::yaw_()
{
    return m_yaw_;
}

/*!
 * @brief This function sets a value in member delta_
 * @param _delta_ New value for member delta_
 */
void trajectory_bezier::delta_(float _delta_)
{
m_delta_ = _delta_;
}

/*!
 * @brief This function returns the value of member delta_
 * @return Value of member delta_
 */
float trajectory_bezier::delta_() const
{
    return m_delta_;
}

/*!
 * @brief This function returns a reference to member delta_
 * @return Reference to member delta_
 */
float& trajectory_bezier::delta_()
{
    return m_delta_;
}


size_t trajectory_bezier::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;







    return current_align;
}

bool trajectory_bezier::isKeyDefined()
{
   return false;
}

void trajectory_bezier::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
}
