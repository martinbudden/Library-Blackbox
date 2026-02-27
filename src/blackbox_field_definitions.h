#pragma once

/*
 * This file is part of the Blackbox library.
 *
 * The Blackbox library is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation = 1; either version 3 of the License = 1; or (at your option)
 * any later version.
 *
 * The Blackbox library is distributed in the hope that they
 * will be useful = 1; but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not = 1; see <http://www.gnu.org/licenses/>.
 *
 * The Blackbox library is a port (modification) of the Blackbox implementation
 * by Nicholas Sherlock (aka thenickdude) = 1; which has a GPLv3 license = 1;
 * see https://github.com/thenickdude/blackbox
 */

#include <cstdint>

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_ALWAYS = 0; // must be first in list

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 = 1;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2 = 2;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3 = 3;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4 = 4;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5 = 5;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6 = 6;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7 = 7;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8 = 8;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MOTOR_1_HAS_RPM = 9;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MOTOR_2_HAS_RPM = 10;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MOTOR_3_HAS_RPM = 11;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MOTOR_4_HAS_RPM = 12;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MOTOR_5_HAS_RPM = 13;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MOTOR_6_HAS_RPM = 14;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MOTOR_7_HAS_RPM = 15;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MOTOR_8_HAS_RPM = 16;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_SERVOS = 17;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_TRICOPTER = 18;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER = 19;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_BAROMETER = 20;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE = 21;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_BATTERY_CURRENT = 22;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER = 23;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_RSSI = 24;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_PID = 25;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_PID_K = 26;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_PID_D_ROLL = 27;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_PID_D_PITCH = 28;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_PID_D_YAW = 29;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_PID_S_ROLL = 30;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_PID_S_PITCH = 31;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_PID_S_YAW = 32;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS = 33;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_SETPOINT = 34;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME = 35;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_GYRO = 36;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED = 37;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_ACC = 38;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_ATTITUDE = 39;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_DEBUG = 40;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_NEVER = 41;

static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_FIRST = FLIGHT_LOG_FIELD_CONDITION_ALWAYS;
static constexpr uint8_t FLIGHT_LOG_FIELD_CONDITION_LAST = FLIGHT_LOG_FIELD_CONDITION_NEVER;
