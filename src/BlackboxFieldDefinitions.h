#pragma once

/*
 * This file is part of the Blackbox library.
 *
 * The Blackbox library is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * The Blackbox library is distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * The Blackbox library is a port (modification) of the Blackbox implementation
 * in Betaflight (which itself was a port of the Cleanflight implementation).
 *
 * The original Betaflight copyright notice is included below, as per the GNU GPL
 * "keep intact all notices” requirement.
 */

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdint>

enum FlightLogFieldCondition_e : uint8_t {
    FLIGHT_LOG_FIELD_CONDITION_ALWAYS = 0,

    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7,
    FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8,

    FLIGHT_LOG_FIELD_CONDITION_MOTOR_1_HAS_RPM,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_2_HAS_RPM,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_3_HAS_RPM,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_4_HAS_RPM,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_5_HAS_RPM,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_6_HAS_RPM,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_7_HAS_RPM,
    FLIGHT_LOG_FIELD_CONDITION_MOTOR_8_HAS_RPM,

    FLIGHT_LOG_FIELD_CONDITION_SERVOS,
    FLIGHT_LOG_FIELD_CONDITION_TRICOPTER,

    FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER,
    FLIGHT_LOG_FIELD_CONDITION_BAROMETER,
    FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE,
    FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC,
    FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER,
    FLIGHT_LOG_FIELD_CONDITION_RSSI,

    FLIGHT_LOG_FIELD_CONDITION_PID,
    FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0,
    FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1,
    FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2,

    FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS,
    FLIGHT_LOG_FIELD_CONDITION_SETPOINT,

    FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME,

    FLIGHT_LOG_FIELD_CONDITION_GYRO,
    FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED,
    FLIGHT_LOG_FIELD_CONDITION_ACC,
    FLIGHT_LOG_FIELD_CONDITION_DEBUG,

    FLIGHT_LOG_FIELD_CONDITION_NEVER,

    FLIGHT_LOG_FIELD_CONDITION_FIRST = FLIGHT_LOG_FIELD_CONDITION_ALWAYS,
    FLIGHT_LOG_FIELD_CONDITION_LAST = FLIGHT_LOG_FIELD_CONDITION_NEVER
};

enum FlightLogFieldSelect_e { // no more than 32
    FLIGHT_LOG_FIELD_SELECT_PID         = 0x01,
    FLIGHT_LOG_FIELD_SELECT_RC_COMMANDS = 0x02,
    FLIGHT_LOG_FIELD_SELECT_SETPOINT    = 0x04,
    FLIGHT_LOG_FIELD_SELECT_BATTERY     = 0x08,
    FLIGHT_LOG_FIELD_SELECT_MAGNETOMETER = 0x10,
    FLIGHT_LOG_FIELD_SELECT_ALTITUDE    = 0x20,
    FLIGHT_LOG_FIELD_SELECT_RSSI        = 0x40,
    FLIGHT_LOG_FIELD_SELECT_GYRO        = 0x80,
    FLIGHT_LOG_FIELD_SELECT_ACC         = 0x100,
    FLIGHT_LOG_FIELD_SELECT_DEBUG_LOG   = 0x200,
    FLIGHT_LOG_FIELD_SELECT_MOTOR       = 0x400,
    FLIGHT_LOG_FIELD_SELECT_GPS         = 0x800,
    FLIGHT_LOG_FIELD_SELECT_MOTOR_RPM   = 0x1000,
    FLIGHT_LOG_FIELD_SELECT_GYRO_UNFILTERED = 0x2000,
    FLIGHT_LOG_FIELD_SELECT_SERVO       = 0x4000
};

struct flightLogEvent_syncBeep_t {
    uint32_t time;
};

struct flightLogEvent_disarm_t {
    uint32_t reason;
};

struct flightLogEvent_flightMode_t { // New Event Data type
    uint32_t flags;
    uint32_t lastFlags;
};

struct flightLogEvent_inflightAdjustment_t {
    int32_t newValue;
    float newFloatValue;
    uint8_t adjustmentFunction;
    bool floatFlag;
};

struct flightLogEvent_loggingResume_t {
    uint32_t logIteration;
    uint32_t currentTime;
};

enum { FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG = 128 };

union flightLogEventData_u {
    flightLogEvent_syncBeep_t syncBeep;
    flightLogEvent_flightMode_t flightMode; // New event data
    flightLogEvent_disarm_t disarm;
    flightLogEvent_inflightAdjustment_t inflightAdjustment;
    flightLogEvent_loggingResume_t loggingResume;
};

enum FlightLogEvent_e {
    FLIGHT_LOG_EVENT_SYNC_BEEP = 0,
    FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START = 10,   // UNUSED
    FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_RESULT = 11,  // UNUSED
    FLIGHT_LOG_EVENT_AUTOTUNE_TARGETS = 12,       // UNUSED
    FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT = 13,
    FLIGHT_LOG_EVENT_LOGGING_RESUME = 14,
    FLIGHT_LOG_EVENT_DISARM = 15,
    FLIGHT_LOG_EVENT_FLIGHTMODE = 30, // Add new event type for flight mode status.
    FLIGHT_LOG_EVENT_LOG_END = 255
};

struct flightLogEvent_t {
    FlightLogEvent_e event;
    flightLogEventData_u data;
};
