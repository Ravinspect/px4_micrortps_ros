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
 * @file battery_status.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _BATTERY_STATUS_H_
#define _BATTERY_STATUS_H_

// TODO Poner en el contexto.

#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(battery_status_SOURCE)
#define battery_status_DllAPI __declspec( dllexport )
#else
#define battery_status_DllAPI __declspec( dllimport )
#endif // battery_status_SOURCE
#else
#define battery_status_DllAPI
#endif
#else
#define battery_status_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}


const uint8_t battery_status__BATTERY_SOURCE_POWER_MODULE = 0;
const uint8_t battery_status__BATTERY_SOURCE_EXTERNAL = 1;
const uint8_t battery_status__BATTERY_SOURCE_ESCS = 2;
const uint8_t battery_status__BATTERY_WARNING_NONE = 0;
const uint8_t battery_status__BATTERY_WARNING_LOW = 1;
const uint8_t battery_status__BATTERY_WARNING_CRITICAL = 2;
const uint8_t battery_status__BATTERY_WARNING_EMERGENCY = 3;
const uint8_t battery_status__BATTERY_WARNING_FAILED = 4;
const uint8_t battery_status__MAX_INSTANCES = 4;
typedef std::array<float, 14> battery_status__float_array_14;
/*!
 * @brief This class represents the structure battery_status defined by the user in the IDL file.
 * @ingroup BATTERY_STATUS
 */
class battery_status
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport battery_status();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~battery_status();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object battery_status that will be copied.
     */
    eProsima_user_DllExport battery_status(const battery_status &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object battery_status that will be copied.
     */
    eProsima_user_DllExport battery_status(battery_status &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object battery_status that will be copied.
     */
    eProsima_user_DllExport battery_status& operator=(const battery_status &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object battery_status that will be copied.
     */
    eProsima_user_DllExport battery_status& operator=(battery_status &&x);

    /*!
     * @brief This function sets a value in member timestamp_
     * @param _timestamp_ New value for member timestamp_
     */
    eProsima_user_DllExport void timestamp_(uint64_t _timestamp_);

    /*!
     * @brief This function returns the value of member timestamp_
     * @return Value of member timestamp_
     */
    eProsima_user_DllExport uint64_t timestamp_() const;

    /*!
     * @brief This function returns a reference to member timestamp_
     * @return Reference to member timestamp_
     */
    eProsima_user_DllExport uint64_t& timestamp_();

    /*!
     * @brief This function sets a value in member connected_
     * @param _connected_ New value for member connected_
     */
    eProsima_user_DllExport void connected_(bool _connected_);

    /*!
     * @brief This function returns the value of member connected_
     * @return Value of member connected_
     */
    eProsima_user_DllExport bool connected_() const;

    /*!
     * @brief This function returns a reference to member connected_
     * @return Reference to member connected_
     */
    eProsima_user_DllExport bool& connected_();

    /*!
     * @brief This function sets a value in member voltage_v_
     * @param _voltage_v_ New value for member voltage_v_
     */
    eProsima_user_DllExport void voltage_v_(float _voltage_v_);

    /*!
     * @brief This function returns the value of member voltage_v_
     * @return Value of member voltage_v_
     */
    eProsima_user_DllExport float voltage_v_() const;

    /*!
     * @brief This function returns a reference to member voltage_v_
     * @return Reference to member voltage_v_
     */
    eProsima_user_DllExport float& voltage_v_();

    /*!
     * @brief This function sets a value in member voltage_filtered_v_
     * @param _voltage_filtered_v_ New value for member voltage_filtered_v_
     */
    eProsima_user_DllExport void voltage_filtered_v_(float _voltage_filtered_v_);

    /*!
     * @brief This function returns the value of member voltage_filtered_v_
     * @return Value of member voltage_filtered_v_
     */
    eProsima_user_DllExport float voltage_filtered_v_() const;

    /*!
     * @brief This function returns a reference to member voltage_filtered_v_
     * @return Reference to member voltage_filtered_v_
     */
    eProsima_user_DllExport float& voltage_filtered_v_();

    /*!
     * @brief This function sets a value in member current_a_
     * @param _current_a_ New value for member current_a_
     */
    eProsima_user_DllExport void current_a_(float _current_a_);

    /*!
     * @brief This function returns the value of member current_a_
     * @return Value of member current_a_
     */
    eProsima_user_DllExport float current_a_() const;

    /*!
     * @brief This function returns a reference to member current_a_
     * @return Reference to member current_a_
     */
    eProsima_user_DllExport float& current_a_();

    /*!
     * @brief This function sets a value in member current_filtered_a_
     * @param _current_filtered_a_ New value for member current_filtered_a_
     */
    eProsima_user_DllExport void current_filtered_a_(float _current_filtered_a_);

    /*!
     * @brief This function returns the value of member current_filtered_a_
     * @return Value of member current_filtered_a_
     */
    eProsima_user_DllExport float current_filtered_a_() const;

    /*!
     * @brief This function returns a reference to member current_filtered_a_
     * @return Reference to member current_filtered_a_
     */
    eProsima_user_DllExport float& current_filtered_a_();

    /*!
     * @brief This function sets a value in member current_average_a_
     * @param _current_average_a_ New value for member current_average_a_
     */
    eProsima_user_DllExport void current_average_a_(float _current_average_a_);

    /*!
     * @brief This function returns the value of member current_average_a_
     * @return Value of member current_average_a_
     */
    eProsima_user_DllExport float current_average_a_() const;

    /*!
     * @brief This function returns a reference to member current_average_a_
     * @return Reference to member current_average_a_
     */
    eProsima_user_DllExport float& current_average_a_();

    /*!
     * @brief This function sets a value in member discharged_mah_
     * @param _discharged_mah_ New value for member discharged_mah_
     */
    eProsima_user_DllExport void discharged_mah_(float _discharged_mah_);

    /*!
     * @brief This function returns the value of member discharged_mah_
     * @return Value of member discharged_mah_
     */
    eProsima_user_DllExport float discharged_mah_() const;

    /*!
     * @brief This function returns a reference to member discharged_mah_
     * @return Reference to member discharged_mah_
     */
    eProsima_user_DllExport float& discharged_mah_();

    /*!
     * @brief This function sets a value in member remaining_
     * @param _remaining_ New value for member remaining_
     */
    eProsima_user_DllExport void remaining_(float _remaining_);

    /*!
     * @brief This function returns the value of member remaining_
     * @return Value of member remaining_
     */
    eProsima_user_DllExport float remaining_() const;

    /*!
     * @brief This function returns a reference to member remaining_
     * @return Reference to member remaining_
     */
    eProsima_user_DllExport float& remaining_();

    /*!
     * @brief This function sets a value in member scale_
     * @param _scale_ New value for member scale_
     */
    eProsima_user_DllExport void scale_(float _scale_);

    /*!
     * @brief This function returns the value of member scale_
     * @return Value of member scale_
     */
    eProsima_user_DllExport float scale_() const;

    /*!
     * @brief This function returns a reference to member scale_
     * @return Reference to member scale_
     */
    eProsima_user_DllExport float& scale_();

    /*!
     * @brief This function sets a value in member temperature_
     * @param _temperature_ New value for member temperature_
     */
    eProsima_user_DllExport void temperature_(float _temperature_);

    /*!
     * @brief This function returns the value of member temperature_
     * @return Value of member temperature_
     */
    eProsima_user_DllExport float temperature_() const;

    /*!
     * @brief This function returns a reference to member temperature_
     * @return Reference to member temperature_
     */
    eProsima_user_DllExport float& temperature_();

    /*!
     * @brief This function sets a value in member cell_count_
     * @param _cell_count_ New value for member cell_count_
     */
    eProsima_user_DllExport void cell_count_(int32_t _cell_count_);

    /*!
     * @brief This function returns the value of member cell_count_
     * @return Value of member cell_count_
     */
    eProsima_user_DllExport int32_t cell_count_() const;

    /*!
     * @brief This function returns a reference to member cell_count_
     * @return Reference to member cell_count_
     */
    eProsima_user_DllExport int32_t& cell_count_();

    /*!
     * @brief This function sets a value in member source_
     * @param _source_ New value for member source_
     */
    eProsima_user_DllExport void source_(uint8_t _source_);

    /*!
     * @brief This function returns the value of member source_
     * @return Value of member source_
     */
    eProsima_user_DllExport uint8_t source_() const;

    /*!
     * @brief This function returns a reference to member source_
     * @return Reference to member source_
     */
    eProsima_user_DllExport uint8_t& source_();

    /*!
     * @brief This function sets a value in member priority_
     * @param _priority_ New value for member priority_
     */
    eProsima_user_DllExport void priority_(uint8_t _priority_);

    /*!
     * @brief This function returns the value of member priority_
     * @return Value of member priority_
     */
    eProsima_user_DllExport uint8_t priority_() const;

    /*!
     * @brief This function returns a reference to member priority_
     * @return Reference to member priority_
     */
    eProsima_user_DllExport uint8_t& priority_();

    /*!
     * @brief This function sets a value in member capacity_
     * @param _capacity_ New value for member capacity_
     */
    eProsima_user_DllExport void capacity_(uint16_t _capacity_);

    /*!
     * @brief This function returns the value of member capacity_
     * @return Value of member capacity_
     */
    eProsima_user_DllExport uint16_t capacity_() const;

    /*!
     * @brief This function returns a reference to member capacity_
     * @return Reference to member capacity_
     */
    eProsima_user_DllExport uint16_t& capacity_();

    /*!
     * @brief This function sets a value in member cycle_count_
     * @param _cycle_count_ New value for member cycle_count_
     */
    eProsima_user_DllExport void cycle_count_(uint16_t _cycle_count_);

    /*!
     * @brief This function returns the value of member cycle_count_
     * @return Value of member cycle_count_
     */
    eProsima_user_DllExport uint16_t cycle_count_() const;

    /*!
     * @brief This function returns a reference to member cycle_count_
     * @return Reference to member cycle_count_
     */
    eProsima_user_DllExport uint16_t& cycle_count_();

    /*!
     * @brief This function sets a value in member run_time_to_empty_
     * @param _run_time_to_empty_ New value for member run_time_to_empty_
     */
    eProsima_user_DllExport void run_time_to_empty_(uint16_t _run_time_to_empty_);

    /*!
     * @brief This function returns the value of member run_time_to_empty_
     * @return Value of member run_time_to_empty_
     */
    eProsima_user_DllExport uint16_t run_time_to_empty_() const;

    /*!
     * @brief This function returns a reference to member run_time_to_empty_
     * @return Reference to member run_time_to_empty_
     */
    eProsima_user_DllExport uint16_t& run_time_to_empty_();

    /*!
     * @brief This function sets a value in member average_time_to_empty_
     * @param _average_time_to_empty_ New value for member average_time_to_empty_
     */
    eProsima_user_DllExport void average_time_to_empty_(uint16_t _average_time_to_empty_);

    /*!
     * @brief This function returns the value of member average_time_to_empty_
     * @return Value of member average_time_to_empty_
     */
    eProsima_user_DllExport uint16_t average_time_to_empty_() const;

    /*!
     * @brief This function returns a reference to member average_time_to_empty_
     * @return Reference to member average_time_to_empty_
     */
    eProsima_user_DllExport uint16_t& average_time_to_empty_();

    /*!
     * @brief This function sets a value in member serial_number_
     * @param _serial_number_ New value for member serial_number_
     */
    eProsima_user_DllExport void serial_number_(uint16_t _serial_number_);

    /*!
     * @brief This function returns the value of member serial_number_
     * @return Value of member serial_number_
     */
    eProsima_user_DllExport uint16_t serial_number_() const;

    /*!
     * @brief This function returns a reference to member serial_number_
     * @return Reference to member serial_number_
     */
    eProsima_user_DllExport uint16_t& serial_number_();

    /*!
     * @brief This function sets a value in member manufacture_date_
     * @param _manufacture_date_ New value for member manufacture_date_
     */
    eProsima_user_DllExport void manufacture_date_(uint16_t _manufacture_date_);

    /*!
     * @brief This function returns the value of member manufacture_date_
     * @return Value of member manufacture_date_
     */
    eProsima_user_DllExport uint16_t manufacture_date_() const;

    /*!
     * @brief This function returns a reference to member manufacture_date_
     * @return Reference to member manufacture_date_
     */
    eProsima_user_DllExport uint16_t& manufacture_date_();

    /*!
     * @brief This function sets a value in member state_of_health_
     * @param _state_of_health_ New value for member state_of_health_
     */
    eProsima_user_DllExport void state_of_health_(uint16_t _state_of_health_);

    /*!
     * @brief This function returns the value of member state_of_health_
     * @return Value of member state_of_health_
     */
    eProsima_user_DllExport uint16_t state_of_health_() const;

    /*!
     * @brief This function returns a reference to member state_of_health_
     * @return Reference to member state_of_health_
     */
    eProsima_user_DllExport uint16_t& state_of_health_();

    /*!
     * @brief This function sets a value in member max_error_
     * @param _max_error_ New value for member max_error_
     */
    eProsima_user_DllExport void max_error_(uint16_t _max_error_);

    /*!
     * @brief This function returns the value of member max_error_
     * @return Value of member max_error_
     */
    eProsima_user_DllExport uint16_t max_error_() const;

    /*!
     * @brief This function returns a reference to member max_error_
     * @return Reference to member max_error_
     */
    eProsima_user_DllExport uint16_t& max_error_();

    /*!
     * @brief This function sets a value in member id_
     * @param _id_ New value for member id_
     */
    eProsima_user_DllExport void id_(uint8_t _id_);

    /*!
     * @brief This function returns the value of member id_
     * @return Value of member id_
     */
    eProsima_user_DllExport uint8_t id_() const;

    /*!
     * @brief This function returns a reference to member id_
     * @return Reference to member id_
     */
    eProsima_user_DllExport uint8_t& id_();

    /*!
     * @brief This function sets a value in member interface_error_
     * @param _interface_error_ New value for member interface_error_
     */
    eProsima_user_DllExport void interface_error_(uint16_t _interface_error_);

    /*!
     * @brief This function returns the value of member interface_error_
     * @return Value of member interface_error_
     */
    eProsima_user_DllExport uint16_t interface_error_() const;

    /*!
     * @brief This function returns a reference to member interface_error_
     * @return Reference to member interface_error_
     */
    eProsima_user_DllExport uint16_t& interface_error_();

    /*!
     * @brief This function copies the value in member voltage_cell_v
     * @param _voltage_cell_v New value to be copied in member voltage_cell_v
     */
    eProsima_user_DllExport void voltage_cell_v(const battery_status__float_array_14 &_voltage_cell_v);

    /*!
     * @brief This function moves the value in member voltage_cell_v
     * @param _voltage_cell_v New value to be moved in member voltage_cell_v
     */
    eProsima_user_DllExport void voltage_cell_v(battery_status__float_array_14 &&_voltage_cell_v);

    /*!
     * @brief This function returns a constant reference to member voltage_cell_v
     * @return Constant reference to member voltage_cell_v
     */
    eProsima_user_DllExport const battery_status__float_array_14& voltage_cell_v() const;

    /*!
     * @brief This function returns a reference to member voltage_cell_v
     * @return Reference to member voltage_cell_v
     */
    eProsima_user_DllExport battery_status__float_array_14& voltage_cell_v();
    /*!
     * @brief This function sets a value in member max_cell_voltage_delta_
     * @param _max_cell_voltage_delta_ New value for member max_cell_voltage_delta_
     */
    eProsima_user_DllExport void max_cell_voltage_delta_(float _max_cell_voltage_delta_);

    /*!
     * @brief This function returns the value of member max_cell_voltage_delta_
     * @return Value of member max_cell_voltage_delta_
     */
    eProsima_user_DllExport float max_cell_voltage_delta_() const;

    /*!
     * @brief This function returns a reference to member max_cell_voltage_delta_
     * @return Reference to member max_cell_voltage_delta_
     */
    eProsima_user_DllExport float& max_cell_voltage_delta_();

    /*!
     * @brief This function sets a value in member is_powering_off_
     * @param _is_powering_off_ New value for member is_powering_off_
     */
    eProsima_user_DllExport void is_powering_off_(bool _is_powering_off_);

    /*!
     * @brief This function returns the value of member is_powering_off_
     * @return Value of member is_powering_off_
     */
    eProsima_user_DllExport bool is_powering_off_() const;

    /*!
     * @brief This function returns a reference to member is_powering_off_
     * @return Reference to member is_powering_off_
     */
    eProsima_user_DllExport bool& is_powering_off_();

    /*!
     * @brief This function sets a value in member warning_
     * @param _warning_ New value for member warning_
     */
    eProsima_user_DllExport void warning_(uint8_t _warning_);

    /*!
     * @brief This function returns the value of member warning_
     * @return Value of member warning_
     */
    eProsima_user_DllExport uint8_t warning_() const;

    /*!
     * @brief This function returns a reference to member warning_
     * @return Reference to member warning_
     */
    eProsima_user_DllExport uint8_t& warning_();

    /*!
     * @brief This function sets a value in member average_power_
     * @param _average_power_ New value for member average_power_
     */
    eProsima_user_DllExport void average_power_(float _average_power_);

    /*!
     * @brief This function returns the value of member average_power_
     * @return Value of member average_power_
     */
    eProsima_user_DllExport float average_power_() const;

    /*!
     * @brief This function returns a reference to member average_power_
     * @return Reference to member average_power_
     */
    eProsima_user_DllExport float& average_power_();

    /*!
     * @brief This function sets a value in member available_energy_
     * @param _available_energy_ New value for member available_energy_
     */
    eProsima_user_DllExport void available_energy_(float _available_energy_);

    /*!
     * @brief This function returns the value of member available_energy_
     * @return Value of member available_energy_
     */
    eProsima_user_DllExport float available_energy_() const;

    /*!
     * @brief This function returns a reference to member available_energy_
     * @return Reference to member available_energy_
     */
    eProsima_user_DllExport float& available_energy_();

    /*!
     * @brief This function sets a value in member remaining_capacity_
     * @param _remaining_capacity_ New value for member remaining_capacity_
     */
    eProsima_user_DllExport void remaining_capacity_(float _remaining_capacity_);

    /*!
     * @brief This function returns the value of member remaining_capacity_
     * @return Value of member remaining_capacity_
     */
    eProsima_user_DllExport float remaining_capacity_() const;

    /*!
     * @brief This function returns a reference to member remaining_capacity_
     * @return Reference to member remaining_capacity_
     */
    eProsima_user_DllExport float& remaining_capacity_();

    /*!
     * @brief This function sets a value in member design_capacity_
     * @param _design_capacity_ New value for member design_capacity_
     */
    eProsima_user_DllExport void design_capacity_(float _design_capacity_);

    /*!
     * @brief This function returns the value of member design_capacity_
     * @return Value of member design_capacity_
     */
    eProsima_user_DllExport float design_capacity_() const;

    /*!
     * @brief This function returns a reference to member design_capacity_
     * @return Reference to member design_capacity_
     */
    eProsima_user_DllExport float& design_capacity_();

    /*!
     * @brief This function sets a value in member average_time_to_full_
     * @param _average_time_to_full_ New value for member average_time_to_full_
     */
    eProsima_user_DllExport void average_time_to_full_(uint16_t _average_time_to_full_);

    /*!
     * @brief This function returns the value of member average_time_to_full_
     * @return Value of member average_time_to_full_
     */
    eProsima_user_DllExport uint16_t average_time_to_full_() const;

    /*!
     * @brief This function returns a reference to member average_time_to_full_
     * @return Reference to member average_time_to_full_
     */
    eProsima_user_DllExport uint16_t& average_time_to_full_();

    /*!
     * @brief This function sets a value in member over_discharge_count_
     * @param _over_discharge_count_ New value for member over_discharge_count_
     */
    eProsima_user_DllExport void over_discharge_count_(uint16_t _over_discharge_count_);

    /*!
     * @brief This function returns the value of member over_discharge_count_
     * @return Value of member over_discharge_count_
     */
    eProsima_user_DllExport uint16_t over_discharge_count_() const;

    /*!
     * @brief This function returns a reference to member over_discharge_count_
     * @return Reference to member over_discharge_count_
     */
    eProsima_user_DllExport uint16_t& over_discharge_count_();

    /*!
     * @brief This function sets a value in member nominal_voltage_
     * @param _nominal_voltage_ New value for member nominal_voltage_
     */
    eProsima_user_DllExport void nominal_voltage_(float _nominal_voltage_);

    /*!
     * @brief This function returns the value of member nominal_voltage_
     * @return Value of member nominal_voltage_
     */
    eProsima_user_DllExport float nominal_voltage_() const;

    /*!
     * @brief This function returns a reference to member nominal_voltage_
     * @return Reference to member nominal_voltage_
     */
    eProsima_user_DllExport float& nominal_voltage_();


    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(const battery_status& data, size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;

private:
    uint64_t m_timestamp_;
    bool m_connected_;
    float m_voltage_v_;
    float m_voltage_filtered_v_;
    float m_current_a_;
    float m_current_filtered_a_;
    float m_current_average_a_;
    float m_discharged_mah_;
    float m_remaining_;
    float m_scale_;
    float m_temperature_;
    int32_t m_cell_count_;
    uint8_t m_source_;
    uint8_t m_priority_;
    uint16_t m_capacity_;
    uint16_t m_cycle_count_;
    uint16_t m_run_time_to_empty_;
    uint16_t m_average_time_to_empty_;
    uint16_t m_serial_number_;
    uint16_t m_manufacture_date_;
    uint16_t m_state_of_health_;
    uint16_t m_max_error_;
    uint8_t m_id_;
    uint16_t m_interface_error_;
    battery_status__float_array_14 m_voltage_cell_v;
    float m_max_cell_voltage_delta_;
    bool m_is_powering_off_;
    uint8_t m_warning_;
    float m_average_power_;
    float m_available_energy_;
    float m_remaining_capacity_;
    float m_design_capacity_;
    uint16_t m_average_time_to_full_;
    uint16_t m_over_discharge_count_;
    float m_nominal_voltage_;
};

#endif // _BATTERY_STATUS_H_