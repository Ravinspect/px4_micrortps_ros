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
 * @file debug_array.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "debug_array.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>




debug_array::debug_array()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@166fa74d
    m_timestamp_ = 0;
    // m_id_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@40f08448
    m_id_ = 0;
    // m_name com.eprosima.idl.parser.typecode.AliasTypeCode@276438c9
    memset(&m_name, 0, (10) * 1);
    // m_data com.eprosima.idl.parser.typecode.AliasTypeCode@588df31b
    memset(&m_data, 0, (58) * 4);

}

debug_array::~debug_array()
{




}

debug_array::debug_array(const debug_array &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_id_ = x.m_id_;
    m_name = x.m_name;
    m_data = x.m_data;
}

debug_array::debug_array(debug_array &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_id_ = x.m_id_;
    m_name = std::move(x.m_name);
    m_data = std::move(x.m_data);
}

debug_array& debug_array::operator=(const debug_array &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_id_ = x.m_id_;
    m_name = x.m_name;
    m_data = x.m_data;

    return *this;
}

debug_array& debug_array::operator=(debug_array &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_id_ = x.m_id_;
    m_name = std::move(x.m_name);
    m_data = std::move(x.m_data);

    return *this;
}

size_t debug_array::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += ((10) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += ((58) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t debug_array::getCdrSerializedSize(const debug_array& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    if ((10) > 0)
    {
        current_alignment += ((10) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }

    if ((58) > 0)
    {
        current_alignment += ((58) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }


    return current_alignment - initial_alignment;
}

void debug_array::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_id_;
    scdr << m_name;

    scdr << m_data;

}

void debug_array::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_id_;
    dcdr >> m_name;

    dcdr >> m_data;

}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void debug_array::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t debug_array::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& debug_array::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function sets a value in member id_
 * @param _id_ New value for member id_
 */
void debug_array::id_(uint16_t _id_)
{
m_id_ = _id_;
}

/*!
 * @brief This function returns the value of member id_
 * @return Value of member id_
 */
uint16_t debug_array::id_() const
{
    return m_id_;
}

/*!
 * @brief This function returns a reference to member id_
 * @return Reference to member id_
 */
uint16_t& debug_array::id_()
{
    return m_id_;
}

/*!
 * @brief This function copies the value in member name
 * @param _name New value to be copied in member name
 */
void debug_array::name(const debug_array__char_array_10 &_name)
{
m_name = _name;
}

/*!
 * @brief This function moves the value in member name
 * @param _name New value to be moved in member name
 */
void debug_array::name(debug_array__char_array_10 &&_name)
{
m_name = std::move(_name);
}

/*!
 * @brief This function returns a constant reference to member name
 * @return Constant reference to member name
 */
const debug_array__char_array_10& debug_array::name() const
{
    return m_name;
}

/*!
 * @brief This function returns a reference to member name
 * @return Reference to member name
 */
debug_array__char_array_10& debug_array::name()
{
    return m_name;
}
/*!
 * @brief This function copies the value in member data
 * @param _data New value to be copied in member data
 */
void debug_array::data(const debug_array__float_array_58 &_data)
{
m_data = _data;
}

/*!
 * @brief This function moves the value in member data
 * @param _data New value to be moved in member data
 */
void debug_array::data(debug_array__float_array_58 &&_data)
{
m_data = std::move(_data);
}

/*!
 * @brief This function returns a constant reference to member data
 * @return Constant reference to member data
 */
const debug_array__float_array_58& debug_array::data() const
{
    return m_data;
}

/*!
 * @brief This function returns a reference to member data
 * @return Reference to member data
 */
debug_array__float_array_58& debug_array::data()
{
    return m_data;
}

size_t debug_array::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;







    return current_align;
}

bool debug_array::isKeyDefined()
{
   return false;
}

void debug_array::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
}
