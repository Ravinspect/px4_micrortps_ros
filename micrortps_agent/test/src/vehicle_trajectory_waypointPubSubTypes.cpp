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
 * @file vehicle_trajectory_waypointPubSubTypes.cpp
 * This header file contains the implementation of the serialization functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include "vehicle_trajectory_waypointPubSubTypes.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;










vehicle_trajectory_waypointPubSubType::vehicle_trajectory_waypointPubSubType()
{
    setName("vehicle_trajectory_waypoint");
    m_typeSize = static_cast<uint32_t>(vehicle_trajectory_waypoint::getMaxCdrSerializedSize()) + 4 /*encapsulation*/;
    m_isGetKeyDefined = vehicle_trajectory_waypoint::isKeyDefined();
    size_t keyLength = vehicle_trajectory_waypoint::getKeyMaxCdrSerializedSize()>16 ? vehicle_trajectory_waypoint::getKeyMaxCdrSerializedSize() : 16;
    m_keyBuffer = reinterpret_cast<unsigned char*>(malloc(keyLength));
    memset(m_keyBuffer, 0, keyLength);
}

vehicle_trajectory_waypointPubSubType::~vehicle_trajectory_waypointPubSubType()
{
    if(m_keyBuffer!=nullptr)
        free(m_keyBuffer);
}

bool vehicle_trajectory_waypointPubSubType::serialize(void *data, SerializedPayload_t *payload)
{
    vehicle_trajectory_waypoint *p_type = static_cast<vehicle_trajectory_waypoint*>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->max_size); // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::Cdr::DDS_CDR); // Object that serializes the data.
    payload->encapsulation = ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    // Serialize encapsulation
    ser.serialize_encapsulation();

    try
    {
        p_type->serialize(ser); // Serialize the object:
    }
    catch(eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/)
    {
        return false;
    }

    payload->length = static_cast<uint32_t>(ser.getSerializedDataLength()); //Get the serialized length
    return true;
}

bool vehicle_trajectory_waypointPubSubType::deserialize(SerializedPayload_t* payload, void* data)
{
    vehicle_trajectory_waypoint* p_type = static_cast<vehicle_trajectory_waypoint*>(data); //Convert DATA to pointer of your type
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->length); // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::Cdr::DDS_CDR); // Object that deserializes the data.
    // Deserialize encapsulation.
    deser.read_encapsulation();
    payload->encapsulation = deser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;

    try
    {
        p_type->deserialize(deser); //Deserialize the object:
    }
    catch(eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/)
    {
        return false;
    }

    return true;
}

std::function<uint32_t()> vehicle_trajectory_waypointPubSubType::getSerializedSizeProvider(void* data)
{
    return [data]() -> uint32_t
    {
        return static_cast<uint32_t>(type::getCdrSerializedSize(*static_cast<vehicle_trajectory_waypoint*>(data))) + 4 /*encapsulation*/;
    };
}

void* vehicle_trajectory_waypointPubSubType::createData()
{
    return reinterpret_cast<void*>(new vehicle_trajectory_waypoint());
}

void vehicle_trajectory_waypointPubSubType::deleteData(void* data)
{
    delete(reinterpret_cast<vehicle_trajectory_waypoint*>(data));
}

bool vehicle_trajectory_waypointPubSubType::getKey(void *data, InstanceHandle_t* handle, bool force_md5)
{
    if(!m_isGetKeyDefined)
        return false;
    vehicle_trajectory_waypoint* p_type = static_cast<vehicle_trajectory_waypoint*>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(m_keyBuffer),vehicle_trajectory_waypoint::getKeyMaxCdrSerializedSize());     // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS);     // Object that serializes the data.
    p_type->serializeKey(ser);
    if(force_md5 || vehicle_trajectory_waypoint::getKeyMaxCdrSerializedSize()>16)    {
        m_md5.init();
        m_md5.update(m_keyBuffer, static_cast<unsigned int>(ser.getSerializedDataLength()));
        m_md5.finalize();
        for(uint8_t i = 0;i<16;++i)        {
            handle->value[i] = m_md5.digest[i];
        }
    }
    else    {
        for(uint8_t i = 0;i<16;++i)        {
            handle->value[i] = m_keyBuffer[i];
        }
    }
    return true;
}
