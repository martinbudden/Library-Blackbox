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
 * by Nicholas Sherlock (aka thenickdude), which has a GPLv3 license,
 * see https://github.com/thenickdude/blackbox
 */

#include <array>
#include <cstdint>

struct blackbox_parameter_group_t;


#pragma pack(push, 1)
// We pack this struct so that padding doesn't interfere with memcmp()
// This data is updated really infrequently:
struct blackbox_slow_state_t {
    uint32_t flight_mode_flags;
    uint8_t state_flags;
    uint8_t failsafe_phase;
    bool rx_signal_received;
    bool rx_flight_channel_is_valid;
};
#pragma pack(pop)

struct blackbox_gps_state_t {
    uint32_t time_of_week_ms;         // GPS time of week in ms
    uint32_t interval_ms;           // interval between GPS solutions in ms
    int32_t home_longitude_degrees1E7; // home longitude in degrees * 1e+7
    int32_t home_latitude_degrees1E7;  // home latitude in degrees * 1e+7
    int32_t home_altitude_cm;        // home altitude in cm
    int32_t longitude_degrees1E7;   // longitude in degrees * 1e+7
    int32_t latitude_degrees1E7;    // latitude in degrees * 1e+7
    int32_t altitude_cm;            // altitude in cm
    int16_t velocity_north_cmps;     // north velocity, cm/s
    int16_t velocity_east_cmps;      // east velocity, cm/s
    int16_t velocity_down_cmps;      // down velocity, cm/s
    int16_t speed3d_cmps;           // speed in cm/s
    int16_t ground_speed_cmps;       // speed in cm/s
    int16_t ground_course_deci_degrees;  // Heading 2D in 10ths of a degree
    uint8_t satellite_count;
};

struct blackbox_main_state_t {
    enum { XYZ_AXIS_COUNT = 3 };
    enum { RPY_AXIS_COUNT = 3 };
#if defined(LIBRARY_BLACKBOX_USE_EIGHT_MOTORS)
    enum { MAX_SUPPORTED_MOTOR_COUNT = 8 };
#else
    enum { MAX_SUPPORTED_MOTOR_COUNT = 4 };
#endif
    enum { MAX_SUPPORTED_SERVO_COUNT = 4 };
    enum { DEBUG_VALUE_COUNT = 8 };

    uint32_t time_us;
    int32_t baro_altitude;
    int32_t surface_raw;
    int32_t amperage_latest;
    uint16_t vbat_latest;
    uint16_t rssi;

    std::array<int32_t, RPY_AXIS_COUNT> axis_pid_p;
    std::array<int32_t, RPY_AXIS_COUNT> axis_pid_i;
    std::array<int32_t, RPY_AXIS_COUNT> axis_pid_d;
    std::array<int32_t, RPY_AXIS_COUNT> axis_pid_s;
    std::array<int32_t, RPY_AXIS_COUNT> axis_pid_k;

    std::array<int16_t, 4> rc_command;
    std::array<int16_t, 4> setpoint;

    std::array<int16_t, XYZ_AXIS_COUNT> gyro_adc;
    std::array<int16_t, XYZ_AXIS_COUNT> gyro_unfiltered;
    std::array<int16_t, XYZ_AXIS_COUNT> acc_adc;
    std::array<int16_t, XYZ_AXIS_COUNT> orientation; // only x,y,z are stored; w is always positive
    std::array<int16_t, XYZ_AXIS_COUNT> mag_adc;

    std::array<int16_t, MAX_SUPPORTED_MOTOR_COUNT> motor;
    std::array<int16_t, MAX_SUPPORTED_MOTOR_COUNT> erpm;
#if defined(LIBRARY_BLACKBOX_USE_SERVOS)
    std::array<int16_t, MAX_SUPPORTED_SERVO_COUNT> servo;
#endif

    std::array<int16_t, DEBUG_VALUE_COUNT> debug;
};

class BlackboxCallbacksBase {
public:
    virtual ~BlackboxCallbacksBase() = default;
    //! Load the rarely-changing values, used for slow frames
    virtual void load_slow_state(blackbox_slow_state_t& slowState, blackbox_parameter_group_t& pg) = 0;

    //! Load the main state of the blackbox, used for I-frames and P-frames
    virtual void load_main_state(blackbox_main_state_t& mainState, uint32_t currentTimeUs, blackbox_parameter_group_t& pg) = 0;
    virtual void load_gps_state(blackbox_gps_state_t& gpsState, blackbox_parameter_group_t& pg) = 0;

    virtual bool is_armed(blackbox_parameter_group_t& pg) const = 0;
    virtual bool is_blackbox_mode_active(blackbox_parameter_group_t& pg) const = 0;
    virtual bool is_blackbox_erase_mode_active(blackbox_parameter_group_t& pg) const = 0;
    virtual bool is_blackbox_mode_activation_condition_present(blackbox_parameter_group_t& pg) const = 0;
    virtual uint32_t get_arming_beep_time_microseconds(blackbox_parameter_group_t& pg) const = 0;
    virtual bool are_motors_running(blackbox_parameter_group_t& pg) const = 0;
    virtual uint32_t rc_mode_activation_mask(blackbox_parameter_group_t& pg) const = 0; // lower 32 bits of BOX_COUNT bits
    virtual void beep(blackbox_parameter_group_t& pg) const = 0;
};
