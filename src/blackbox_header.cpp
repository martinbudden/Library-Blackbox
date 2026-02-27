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

#include "blackbox.h"
#include "blackbox_header.h"
#include "blackbox_serial_device.h"
#include "printf.h"
#include <cassert>
#include <cstring>

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-type-union-access,readability-magic-numbers)
const std::array<char, 79> blackboxHeader = {
    "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    "H Data version:2\n"
};

enum { BLACKBOX_FIELD_HEADER_NAMES_COUNT = 6 };
enum {
    BLACKBOX_DELTA_FIELD_HEADER_COUNT       = BLACKBOX_FIELD_HEADER_NAMES_COUNT,
    BLACKBOX_SIMPLE_FIELD_HEADER_COUNT      = BLACKBOX_FIELD_HEADER_NAMES_COUNT - 2,
    BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT = BLACKBOX_FIELD_HEADER_NAMES_COUNT - 2
};

const std::array<const char* const, BLACKBOX_FIELD_HEADER_NAMES_COUNT> blackboxFieldHeaderNames = {
    "name",
    "signed",
    "predictor",
    "encoding",
    "predictor",
    "encoding"
};

// Some macros to make writing FLIGHT_LOG_FIELD_* constants shorter:
// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define CONCAT_HELPER(x,y) x ## y
#define CONCAT(x,y) CONCAT_HELPER(x, y)
//#define CONCAT2(_1,_2) CONCAT(_1, _2)
//#define CONCAT3(_1,_2,_3)  CONCAT(CONCAT(_1, _2), _3)
//#define CONCAT4(_1,_2,_3,_4)  CONCAT(CONCAT3(_1, _2, _3), _4)

#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED
#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
// NOLINTEND(cppcoreguidelines-macro-usage)

// Rarely-updated fields
enum { SLOW_FIELD_COUNT = 5 };
static const std::array<blackbox_simple_field_definition_t, SLOW_FIELD_COUNT> blackboxSlowFields={{
    {.name="flight_mode_flags",       .field_name_index=-1,.is_signed=UNSIGNED, .predict=PREDICT(0), .encode=ENCODING(UNSIGNED_VB)},
    {.name="state_flags",            .field_name_index=-1,.is_signed=UNSIGNED, .predict=PREDICT(0), .encode=ENCODING(UNSIGNED_VB)},

    {.name="failsafe_phase",         .field_name_index=-1,.is_signed=UNSIGNED, .predict=PREDICT(0), .encode=ENCODING(TAG2_3S32)},
    {.name="rx_signal_received",      .field_name_index=-1,.is_signed=UNSIGNED, .predict=PREDICT(0), .encode=ENCODING(TAG2_3S32)},
    {.name="rx_flight_channe_is_valid", .field_name_index=-1,.is_signed=UNSIGNED, .predict=PREDICT(0), .encode=ENCODING(TAG2_3S32)}
}};

#if defined(LIBRARY_BLACKBOX_USE_GPS)
// GPS home frame
enum { GPS_H_FIELD_COUNT = 3 };
static const std::array<blackbox_simple_field_definition_t, GPS_H_FIELD_COUNT> blackboxGpsHFields={{
    {.name="GPS_home",      .field_name_index=0,  .is_signed=SIGNED,     .predict=PREDICT(0),      .encode=ENCODING(SIGNED_VB)},
    {.name="GPS_home",      .field_name_index=1,  .is_signed=SIGNED,     .predict=PREDICT(0),      .encode=ENCODING(SIGNED_VB)},
    {.name="GPS_home",      .field_name_index=2,  .is_signed=SIGNED,     .predict=PREDICT(0),      .encode=ENCODING(SIGNED_VB)}
}};

// GPS position/velocity frame
enum { GPS_G_FIELD_COUNT = 10 };
static const std::array<blackbox_conditional_field_definition_t, GPS_G_FIELD_COUNT> blackboxGpsGFields={{
    {.name="time",          .field_name_index=-1, .is_signed=UNSIGNED,  .predict=PREDICT(LAST_MAIN_FRAME_TIME), .encode=ENCODING(UNSIGNED_VB),.condition=CONDITION(NOT_LOGGING_EVERY_FRAME)},
    {.name="GPS_numSat",    .field_name_index=-1, .is_signed=UNSIGNED,  .predict=PREDICT(0),       .encode=ENCODING(UNSIGNED_VB),  .condition=CONDITION(ALWAYS)},
    {.name="GPS_coord",     .field_name_index=0,  .is_signed=SIGNED,    .predict=PREDICT(HOME_COORD),.encode=ENCODING(SIGNED_VB),  .condition=CONDITION(ALWAYS)},
    {.name="GPS_coord",     .field_name_index=1,  .is_signed=SIGNED,    .predict=PREDICT(HOME_COORD),.encode=ENCODING(SIGNED_VB),  .condition=CONDITION(ALWAYS)},
    {.name="GPS_altitude",  .field_name_index=-1, .is_signed=SIGNED,    .predict=PREDICT(0),       .encode=ENCODING(SIGNED_VB),    .condition=CONDITION(ALWAYS)},
    {.name="GPS_speed",     .field_name_index=-1, .is_signed=UNSIGNED,  .predict=PREDICT(0),       .encode=ENCODING(UNSIGNED_VB),  .condition=CONDITION(ALWAYS)},
    {.name="GPS_ground_course",.field_name_index=-1,.is_signed=UNSIGNED,.predict=PREDICT(0),       .encode=ENCODING(UNSIGNED_VB),  .condition=CONDITION(ALWAYS)},
    {.name="GPS_velned",    .field_name_index=0,  .is_signed=SIGNED,    .predict=PREDICT(0),       .encode=ENCODING(SIGNED_VB),    .condition=CONDITION(ALWAYS)},
    {.name="GPS_velned",    .field_name_index=1,  .is_signed=SIGNED,    .predict=PREDICT(0),       .encode=ENCODING(SIGNED_VB),    .condition=CONDITION(ALWAYS)},
    {.name="GPS_velned",    .field_name_index=2,  .is_signed=SIGNED,    .predict=PREDICT(0),       .encode=ENCODING(SIGNED_VB),    .condition=CONDITION(ALWAYS)},
}};
#endif

/*!
Description of the blackbox fields we are writing in our main I (intra) and P (inter) frames.
This description is written into the flight log header so the log can be properly interpreted.
These definitions don't actually cause the encoding to happen,
we have to encode the flight log ourselves in log_pFrame and log_iFrame in a way that matches the encoding we've promised here).

Field names and encodings are choosen to be compatible with Betaflight blackbox logs.
*/
//static const std::array<Blackbox::blackbox_delta_field_definition_t, MAIN_FIELD_COUNT> blackboxMainFields={{
static const blackbox_delta_field_definition_t blackboxMainFields[]={ // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
    /* loopIteration doesn't appear in P frames since it always increments */
    {.name="loopIteration", .field_name_index=-1, .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(INC),         .p_encode=FLIGHT_LOG_FIELD_ENCODING_NULL,.condition=CONDITION(ALWAYS)},
    // Time advances pretty steadily so the P-frame prediction is a straight line
    {.name="time",          .field_name_index=-1, .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(STRAIGHT_LINE),.p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(ALWAYS)},
    {.name="axisP",         .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID)},
    {.name="axisP",         .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID)},
    {.name="axisP",         .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID)},
    // I terms get special packed encoding in P frames:
    {.name="axisI",         .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG2_3S32),.condition=CONDITION(PID)},
    {.name="axisI",         .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG2_3S32),.condition=CONDITION(PID)},
    {.name="axisI",         .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG2_3S32),.condition=CONDITION(PID)},
    {.name="axisD",         .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_D_ROLL)},
    {.name="axisD",         .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_D_PITCH)},
    {.name="axisD",         .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_D_YAW)},
    // PID K terms use F (feedforward) suffix to be compatible with Betaflight blackbox logs
    {.name="axisF",         .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_K)},
    {.name="axisF",         .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_K)},
    {.name="axisF",         .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_K)},
    {.name="axisS",         .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_S_ROLL)},
    {.name="axisS",         .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_S_PITCH)},
    {.name="axisS",         .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(PID_S_YAW)},
    // rc_commands are encoded together as a group in P-frames:
    {.name="rc_command",     .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_4S16),.condition=CONDITION(RC_COMMANDS)},
    {.name="rc_command",     .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_4S16),.condition=CONDITION(RC_COMMANDS)},
    {.name="rc_command",     .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_4S16),.condition=CONDITION(RC_COMMANDS)},
    {.name="rc_command",     .field_name_index=3,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_4S16),.condition=CONDITION(RC_COMMANDS)},
    // setpoint - define 4 fields like rc_command to use the same encoding. setpoint[4] contains the mixer throttle
    {.name="setpoint",      .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_4S16),.condition=CONDITION(SETPOINT)},
    {.name="setpoint",      .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_4S16),.condition=CONDITION(SETPOINT)},
    {.name="setpoint",      .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_4S16),.condition=CONDITION(SETPOINT)},
    {.name="setpoint",      .field_name_index=3,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_4S16),.condition=CONDITION(SETPOINT)},
    {.name="vbat_latest",    .field_name_index=-1, .is_signed=UNSIGNED, .i_predict=PREDICT(VBATREF),.i_encode=ENCODING(NEG_14BIT),.p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(BATTERY_VOLTAGE)},
    {.name="amperage_latest",.field_name_index=-1, .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(BATTERY_CURRENT)},
#if defined(LIBRARY_BLACKBOX_USE_MAGNETOMETER)
    {.name="mag_adc",        .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(MAGNETOMETER)},
    {.name="mag_adc",        .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(MAGNETOMETER)},
    {.name="mag_adc",        .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(MAGNETOMETER)},
#endif
#if defined(LIBRARY_BLACKBOX_USE_BAROMETER)
    {.name="baro_altitude",       .field_name_index=-1, .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(BAROMETER)},
#endif
#if defined(LIBRARY_BLACKBOX_USE_RANGEFINDER)
    {.name="surface_raw",    .field_name_index=-1, .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(RANGEFINDER)},
#endif
    {.name="rssi",          .field_name_index=-1, .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(RSSI)},
    // Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact
    {.name="gyro_adc",       .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(GYRO)},
    {.name="gyro_adc",       .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(GYRO)},
    {.name="gyro_adc",       .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(GYRO)},
    {.name="gyroUnfilt",    .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(GYRO_UNFILTERED)},
    {.name="gyroUnfilt",    .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(GYRO_UNFILTERED)},
    {.name="gyroUnfilt",    .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(GYRO_UNFILTERED)},
    {.name="accSmooth",     .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(ACC)},
    {.name="accSmooth",     .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(ACC)},
    {.name="accSmooth",     .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(ACC)},
    {.name="imuQuaternion", .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(ATTITUDE)},
    {.name="imuQuaternion", .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(ATTITUDE)},
    {.name="imuQuaternion", .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(ATTITUDE)},
    {.name="debug",         .field_name_index=0,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(DEBUG)},
    {.name="debug",         .field_name_index=1,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(DEBUG)},
    {.name="debug",         .field_name_index=2,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(DEBUG)},
    {.name="debug",         .field_name_index=3,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(DEBUG)},
    {.name="debug",         .field_name_index=4,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(DEBUG)},
    {.name="debug",         .field_name_index=5,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(DEBUG)},
    {.name="debug",         .field_name_index=6,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(DEBUG)},
    {.name="debug",         .field_name_index=7,  .is_signed=SIGNED,   .i_predict=PREDICT(0),   .i_encode=ENCODING(SIGNED_VB),   .p_predict=PREDICT(AVERAGE_2),   .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(DEBUG)},
    // Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones):
    // must match with enum { MAX_SUPPORTED_MOTORS=4 };
    {.name="motor",         .field_name_index=0,  .is_signed=UNSIGNED, .i_predict=PREDICT(MINMOTOR), .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(AVERAGE_2),.p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(AT_LEAST_MOTORS_1)},
    /* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
    {.name="motor",         .field_name_index=1,  .is_signed=UNSIGNED, .i_predict=PREDICT(MOTOR_0), .i_encode=ENCODING(SIGNED_VB),.p_predict=PREDICT(AVERAGE_2),  .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(AT_LEAST_MOTORS_2)},
    {.name="motor",         .field_name_index=2,  .is_signed=UNSIGNED, .i_predict=PREDICT(MOTOR_0), .i_encode=ENCODING(SIGNED_VB),.p_predict=PREDICT(AVERAGE_2),  .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(AT_LEAST_MOTORS_3)},
    {.name="motor",         .field_name_index=3,  .is_signed=UNSIGNED, .i_predict=PREDICT(MOTOR_0), .i_encode=ENCODING(SIGNED_VB),.p_predict=PREDICT(AVERAGE_2),  .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(AT_LEAST_MOTORS_4)},
#if defined(LIBRARY_BLACKBOX_USE_EIGHT_MOTORS)
    {.name="motor",         .field_name_index=4,  .is_signed=UNSIGNED, .i_predict=PREDICT(MOTOR_0), .i_encode=ENCODING(SIGNED_VB),.p_predict=PREDICT(AVERAGE_2),  .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(AT_LEAST_MOTORS_5)},
    {.name="motor",         .field_name_index=5,  .is_signed=UNSIGNED, .i_predict=PREDICT(MOTOR_0), .i_encode=ENCODING(SIGNED_VB),.p_predict=PREDICT(AVERAGE_2),  .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(AT_LEAST_MOTORS_6)},
    {.name="motor",         .field_name_index=6,  .is_signed=UNSIGNED, .i_predict=PREDICT(MOTOR_0), .i_encode=ENCODING(SIGNED_VB),.p_predict=PREDICT(AVERAGE_2),  .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(AT_LEAST_MOTORS_7)},
    {.name="motor",         .field_name_index=7,  .is_signed=UNSIGNED, .i_predict=PREDICT(MOTOR_0), .i_encode=ENCODING(SIGNED_VB),.p_predict=PREDICT(AVERAGE_2),  .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(AT_LEAST_MOTORS_8)},
#endif
#if defined(LIBRARY_BLACKBOX_USE_SERVOS)
    // must match with MAX_SUPPORTED_SERVO_COUNT
    // NOTE (ledvinap, hwarhurst): Decoding would fail if previous encoding is also TAG8_8SVB and does not have exactly 8 values. To fix it, inserting ENCODING_NULL dummy value should force end of previous group.
    {.name="servo",         .field_name_index=0,  .is_signed=UNSIGNED, .i_predict=PREDICT(1500),.i_encode=ENCODING(TAG8_8SVB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(SERVOS)},
    {.name="servo",         .field_name_index=1,  .is_signed=UNSIGNED, .i_predict=PREDICT(1500),.i_encode=ENCODING(TAG8_8SVB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(SERVOS)},
    {.name="servo",         .field_name_index=2,  .is_signed=UNSIGNED, .i_predict=PREDICT(1500),.i_encode=ENCODING(TAG8_8SVB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(SERVOS)},
    {.name="servo",         .field_name_index=3,  .is_signed=UNSIGNED, .i_predict=PREDICT(1500),.i_encode=ENCODING(TAG8_8SVB),   .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(TAG8_8SVB),.condition=CONDITION(SERVOS)},
#endif
#if defined(LIBRARY_BLACKBOX_USE_DSHOT_TELEMETRY)
    // must match with MAX_SUPPORTED_MOTOR_COUNT
    // eRPM / 100
    {.name="eRPM",          .field_name_index=0,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(MOTOR_1_HAS_RPM)},
    {.name="eRPM",          .field_name_index=1,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(MOTOR_2_HAS_RPM)},
    {.name="eRPM",          .field_name_index=2,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(MOTOR_3_HAS_RPM)},
    {.name="eRPM",          .field_name_index=3,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(MOTOR_4_HAS_RPM)},
#if defined(LIBRARY_BLACKBOX_USE_EIGHT_MOTORS)
    {.name="eRPM",          .field_name_index=4,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(MOTOR_5_HAS_RPM)},
    {.name="eRPM",          .field_name_index=5,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(MOTOR_6_HAS_RPM)},
    {.name="eRPM",          .field_name_index=6,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(MOTOR_7_HAS_RPM)},
    {.name="eRPM",          .field_name_index=7,  .is_signed=UNSIGNED, .i_predict=PREDICT(0),   .i_encode=ENCODING(UNSIGNED_VB), .p_predict=PREDICT(PREVIOUS),    .p_encode=ENCODING(SIGNED_VB),.condition=CONDITION(MOTOR_8_HAS_RPM)},
#endif
#endif // USE_DSHOT_TELEMETRY
#undef CONDITION
};

size_t Blackbox::printfv(const char* fmt, va_list va)
{
    return tfp_format(&_serial_device, BlackboxEncoder::putc, fmt, va);
}


//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
size_t Blackbox::printf(const char* fmt, ...) // NOLINT(cert-dcl50-cpp)
{
    va_list va;
    va_start(va, fmt);
    const size_t written = printfv(fmt, va);
    va_end(va);

    return written;
}

/*!
`printf` a Blackbox header line with a leading "H " and trailing "\n" added automatically.
`_header_budget` is decreased to account for the number of bytes written.
 */
size_t Blackbox::header_printf_header_line(const char* name, const char* fmt, ...) // NOLINT(cert-dcl50-cpp)
{
    header_write('H');
    header_write(' ');
    header_write_string(name);
    header_write(':');

    va_list va;
    va_start(va, fmt);

    const size_t written = printfv(fmt, va);

    va_end(va);

    header_write('\n');

    _header_budget -= static_cast<int32_t>(written + 3);

    return written + 3;
}

size_t Blackbox::header_printf(const char *fmt, ...) // NOLINT(cert-dcl50-cpp)
{
    va_list va;
    va_start(va, fmt);

    const size_t ret = printfv(fmt, va); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)

    va_end(va);

    return ret;
}

void Blackbox::header_write(uint8_t value)
{
    _encoder.write(value);
}

size_t Blackbox::header_write_string(const char* s)
{
    const auto* pos = reinterpret_cast<const uint8_t*>(s); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    size_t length = 0;
    while (*pos) {
        header_write(*pos);
        ++pos; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ++length;
        //if (length >= _buf.max_size()) {
        //    break;
        //}
    }

    return length;
}

bool Blackbox::header_reserve_buffer_space()
{
    enum { LONGEST_LINE_LENGTH = 64 };
    if (_serial_device.reserve_buffer_space(LONGEST_LINE_LENGTH) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        return false; // NOLINT(readability-simplify-boolean-expr)
    }
    return true;
}


Blackbox::write_e Blackbox::write_header()
{
    // Transmit the header in chunks so we don't overflow its transmit
    // buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
    if (_serial_device.reserve_buffer_space(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        for (size_t ii = 0; ii < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[_xmit_state.headerIndex] != '\0'; ++ii) {
            //Write header, ie
            //"H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
            //"H Data version:2\n"
            _encoder.write(static_cast<uint8_t>(blackboxHeader[_xmit_state.headerIndex]));
            --_header_budget;
            ++_xmit_state.headerIndex;
        }
        if (blackboxHeader[_xmit_state.headerIndex] == 0) {
            return WRITE_COMPLETE; // we have finished
        }
    }
    return WRITE_NOT_COMPLETE; //  we have more to write
}

Blackbox::write_e Blackbox::write_field_header_main() // NOLINT(readability-function-cognitive-complexity)
{
    // We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
    // the whole header.

    // On our first call we need to print the name of the header and a colon
    const int32_t field_count = sizeof(blackboxMainFields) / sizeof(blackbox_delta_field_definition_t);
    if (_xmit_state.fieldIndex == -1) {
        const size_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[_xmit_state.headerIndex]);
        if (_serial_device.reserve_buffer_space(charsToBeWritten) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
            return WRITE_NOT_COMPLETE; // Try again later
        }
        if (_xmit_state.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT) {
            _header_budget -= static_cast<int32_t>(header_printf("H Field P %s:", blackboxFieldHeaderNames[_xmit_state.headerIndex])); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        } else {
            _header_budget -= static_cast<int32_t>(header_printf("H Field I %s:", blackboxFieldHeaderNames[_xmit_state.headerIndex])); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        }
        ++_xmit_state.fieldIndex;
    }
    if (_xmit_state.headerIndex == 0) {
        //0: H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisD[2],rc_command[0],rc_command[1],rc_command[2],rc_command[3],vbat_latest,amperage_latest,gyro_adc[0],gyro_adc[1],gyro_adc[2],motor[0],motor[1],motor[2],motor[3]
        for (; _xmit_state.fieldIndex < field_count; ++_xmit_state.fieldIndex) {
            const blackbox_delta_field_definition_t& def = blackboxMainFields[_xmit_state.fieldIndex];
            if (test_field_condition(def.condition)) {
                if (def.field_name_index == -1) {
                    header_printf(_xmit_state.fieldIndex == 0 ? "%s" : ",%s", def.name); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
                } else {
                    header_printf(_xmit_state.fieldIndex == 0 ? "%s[%d]" : ",%s[%d]", def.name, def.field_name_index); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
                }
            }
        }
    } else {
        //1: H Field I signed:   0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1, 0,0,0,0
        //2: H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,9,0,0,0,0,11,5,5,5
        //3: H Field I encoding: 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,3,1,0,0,0, 1,0,0,0
        //4: H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3, 3,3,3,3
        //5: H Field P encoding: 9,0,0,0,0,7,7,7,0,0,0,8,8,8,8,6,6,0,0,0, 0,0,0,0
        for (; _xmit_state.fieldIndex < field_count; ++_xmit_state.fieldIndex) {
            const blackbox_delta_field_definition_t& def = blackboxMainFields[_xmit_state.fieldIndex];
            const uint8_t value =
                _xmit_state.headerIndex == 1 ? def.is_signed :
                _xmit_state.headerIndex == 2 ? def.i_predict :
                _xmit_state.headerIndex == 3 ? def.i_encode :
                _xmit_state.headerIndex == 4 ? def.p_predict : def.p_encode;
            if (test_field_condition(def.condition)) {
                header_printf(_xmit_state.fieldIndex == 0 ? "%d" : ",%d", value); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
            }
        }
    }
    if (_xmit_state.fieldIndex == field_count && _serial_device.reserve_buffer_space(1) == BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        --_header_budget;
        header_write('\n');
        ++_xmit_state.headerIndex;
        _xmit_state.fieldIndex = -1; // set fieldIndex to -1 to write the field header next time round
    }

    // return WRITE_COMPLETE if we have nothing more to write
    return _xmit_state.headerIndex < BLACKBOX_DELTA_FIELD_HEADER_COUNT ? WRITE_NOT_COMPLETE : WRITE_COMPLETE;
}

Blackbox::write_e Blackbox::write_field_header_simple(char field_char, const blackbox_simple_field_definition_t* fields, int32_t field_count) // NOLINT(readability-function-cognitive-complexity)
{
    // We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
    // the whole header.

    // On our first call we need to print the name of the header and a colon
    if (_xmit_state.fieldIndex == -1) {
        const size_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[_xmit_state.headerIndex]);
        if (_serial_device.reserve_buffer_space(charsToBeWritten) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
            return WRITE_NOT_COMPLETE; // Try again later
        }
        _header_budget -= static_cast<int32_t>(header_printf("H Field %c %s:", field_char, blackboxFieldHeaderNames[_xmit_state.headerIndex])); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        ++_xmit_state.fieldIndex;
    }
    if (_xmit_state.headerIndex == 0) {
        // H Field S name:flight_mode_flags,state_flags,failsafe_phase,rx_signal_received,rx_flight_channe_is_valid
        // H Field H name:GPS_home[0],GPS_home[1]
        for (; _xmit_state.fieldIndex < field_count; ++_xmit_state.fieldIndex) {
            const blackbox_simple_field_definition_t& def = fields[static_cast<size_t>(_xmit_state.fieldIndex)];
            if (def.field_name_index == -1) {
                header_printf(_xmit_state.fieldIndex == 0 ? "%s" : ",%s", def.name); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
            } else {
                header_printf(_xmit_state.fieldIndex == 0 ? "%s[%d]" : ",%s[%d]", def.name, def.field_name_index); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
            }
        }
    } else {
        //1:H Field S signed:   0,0,0,0,0
        //2:H Field S predictor:0,0,0,0,0
        //3:H Field S encoding: 1,1,7,7,7
        for (; _xmit_state.fieldIndex < field_count; ++_xmit_state.fieldIndex) {
            const blackbox_simple_field_definition_t& def = fields[static_cast<size_t>(_xmit_state.fieldIndex)];
            const uint8_t value =
                _xmit_state.headerIndex == 1 ? def.is_signed :
                _xmit_state.headerIndex == 2 ? def.predict : def.encode;
            header_printf(_xmit_state.fieldIndex == 0 ? "%d" : ",%d", value); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        }
    }
    if (_xmit_state.fieldIndex == field_count && _serial_device.reserve_buffer_space(1) == BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        --_header_budget;
        header_write('\n');
        ++_xmit_state.headerIndex;
        _xmit_state.fieldIndex = -1; // set fieldIndex to -1 to write the field header next time round
    }
    // return WRITE_COMPLETE if we have nothing more to write
    return _xmit_state.headerIndex < BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? WRITE_NOT_COMPLETE : WRITE_COMPLETE;
}

Blackbox::write_e Blackbox::write_field_header_slow()
{
    return write_field_header_simple('S', &blackboxSlowFields[0], blackboxSlowFields.size());
}

Blackbox::write_e Blackbox::write_field_header_gps_h() // NOLINT(readability-convert-member-functions-to-static)
{
#if defined(LIBRARY_BLACKBOX_USE_GPS)
    return write_field_header_simple('H', &blackboxGpsHFields[0], blackboxGpsHFields.size());
#else
    return WRITE_COMPLETE;
#endif
}

Blackbox::write_e Blackbox::write_field_header_gps_g() // NOLINT(readability-convert-member-functions-to-static,readability-function-cognitive-complexity)
{
#if defined(LIBRARY_BLACKBOX_USE_GPS)
    // On our first call we need to print the name of the header and a colon
    const int32_t field_count = blackboxGpsGFields.size();
    if (_xmit_state.fieldIndex == -1) {
        const size_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[_xmit_state.headerIndex]);
        if (_serial_device.reserve_buffer_space(charsToBeWritten) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
            return WRITE_NOT_COMPLETE; // Try again later
        }
        _header_budget -= static_cast<int32_t>(header_printf("H Field G %s:", blackboxFieldHeaderNames[_xmit_state.headerIndex])); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        ++_xmit_state.fieldIndex;
    }
    if (_xmit_state.headerIndex == 0) {
        // H Field G name:time,GPS_numSat,GPS_coord[0],GPS_coord[1],GPS_altitude,GPS_speed,GPS_ground_course,GPS_velned[0],GPS_velned[1],GPS_velned[2]
        for (; _xmit_state.fieldIndex < field_count; ++_xmit_state.fieldIndex) {
            const blackbox_conditional_field_definition_t& def = blackboxGpsGFields[static_cast<size_t>(_xmit_state.fieldIndex)];
            if (def.field_name_index == -1) {
                header_printf(_xmit_state.fieldIndex == 0 ? "%s" : ",%s", def.name); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
            } else {
                header_printf(_xmit_state.fieldIndex == 0 ? "%s[%d]" : ",%s[%d]", def.name, def.field_name_index); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
            }
        }
    } else {
        // H Field G signed:0,0,1,1,1,0,0,1,1,1
        // H Field G predictor:10,0,7,7,0,0,0,0,0,0
        // H Field G encoding:1,1,0,0,0,1,1,0,0,0
        for (; _xmit_state.fieldIndex < field_count; ++_xmit_state.fieldIndex) {
            const blackbox_conditional_field_definition_t& def = blackboxGpsGFields[static_cast<size_t>(_xmit_state.fieldIndex)];
            const uint8_t value =
                _xmit_state.headerIndex == 1 ? def.is_signed :
                _xmit_state.headerIndex == 2 ? def.predict : def.encode;
            header_printf(_xmit_state.fieldIndex == 0 ? "%d" : ",%d", value); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        }
    }
    if (_xmit_state.fieldIndex == field_count && _serial_device.reserve_buffer_space(1) == BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        --_header_budget;
        header_write('\n');
        ++_xmit_state.headerIndex;
        _xmit_state.fieldIndex = -1; // set fieldIndex to -1 to write the field header next time round
    }

    // return WRITE_COMPLETE if we have nothing more to write
    return _xmit_state.headerIndex < BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT ? WRITE_NOT_COMPLETE : WRITE_COMPLETE;
#else
    return WRITE_COMPLETE;
#endif
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-type-union-access,readability-magic-numbers)

/*!
Called from build_field_condition_cache(), which is called from start()
*/
bool Blackbox::test_field_condition_uncached(uint8_t condition) const
{
    switch (condition) {
    case FLIGHT_LOG_FIELD_CONDITION_ALWAYS:
        return true;
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8:
        return is_field_enabled(LOG_SELECT_MOTOR) && (static_cast<int32_t>(_motor_count) > condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1);

    case FLIGHT_LOG_FIELD_CONDITION_MOTOR_1_HAS_RPM:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_MOTOR_2_HAS_RPM:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_MOTOR_3_HAS_RPM:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_MOTOR_4_HAS_RPM:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_MOTOR_5_HAS_RPM:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_MOTOR_6_HAS_RPM:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_MOTOR_7_HAS_RPM:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_MOTOR_8_HAS_RPM:
        return is_field_enabled(LOG_SELECT_MOTOR_RPM) && (static_cast<int32_t>(_motor_count) > condition - FLIGHT_LOG_FIELD_CONDITION_MOTOR_1_HAS_RPM);

    case FLIGHT_LOG_FIELD_CONDITION_SERVOS:
        return is_field_enabled(LOG_SELECT_SERVO) && (_servo_count > 0);

    case FLIGHT_LOG_FIELD_CONDITION_PID:
        return is_field_enabled(LOG_SELECT_PID);

    case FLIGHT_LOG_FIELD_CONDITION_PID_K:
        return is_field_enabled(LOG_SELECT_PID) && is_field_enabled(LOG_SELECT_PID_KTERM);

    case FLIGHT_LOG_FIELD_CONDITION_PID_D_ROLL:
        return is_field_enabled(LOG_SELECT_PID) && is_field_enabled(LOG_SELECT_PID_DTERM_ROLL);
    case FLIGHT_LOG_FIELD_CONDITION_PID_D_PITCH:
        return is_field_enabled(LOG_SELECT_PID) && is_field_enabled(LOG_SELECT_PID_DTERM_PITCH);
    case FLIGHT_LOG_FIELD_CONDITION_PID_D_YAW:
        return is_field_enabled(LOG_SELECT_PID) && is_field_enabled(LOG_SELECT_PID_DTERM_YAW);

    case FLIGHT_LOG_FIELD_CONDITION_PID_S_ROLL:
        return is_field_enabled(LOG_SELECT_PID) && is_field_enabled(LOG_SELECT_PID_STERM_ROLL);
    case FLIGHT_LOG_FIELD_CONDITION_PID_S_PITCH:
        return is_field_enabled(LOG_SELECT_PID) && is_field_enabled(LOG_SELECT_PID_STERM_PITCH);
    case FLIGHT_LOG_FIELD_CONDITION_PID_S_YAW:
        return is_field_enabled(LOG_SELECT_PID) && is_field_enabled(LOG_SELECT_PID_STERM_YAW);

    case FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS:
        return is_field_enabled(LOG_SELECT_RC_COMMANDS);

    case FLIGHT_LOG_FIELD_CONDITION_SETPOINT:
        return is_field_enabled(LOG_SELECT_SETPOINT);

    case FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER:
        return is_field_enabled(LOG_SELECT_MAGNETOMETER);

    case FLIGHT_LOG_FIELD_CONDITION_BAROMETER:
        return is_field_enabled(LOG_SELECT_BAROMETER);

    case FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE:
        return is_field_enabled(LOG_SELECT_BATTERY_VOLTAGE);

    case FLIGHT_LOG_FIELD_CONDITION_BATTERY_CURRENT:
        return is_field_enabled(LOG_SELECT_BATTERY_CURRENT);

    case FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER:
        return is_field_enabled(LOG_SELECT_RANGEFINDER);

    case FLIGHT_LOG_FIELD_CONDITION_RSSI:
        return is_field_enabled(LOG_SELECT_RSSI);

    case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
        return _p_interval != _i_interval;

    case FLIGHT_LOG_FIELD_CONDITION_GYRO:
        return is_field_enabled(LOG_SELECT_GYRO);

    case FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED:
        return is_field_enabled(LOG_SELECT_GYRO_UNFILTERED);

    case FLIGHT_LOG_FIELD_CONDITION_ACC:
        return is_field_enabled(LOG_SELECT_ACCELEROMETER);

    case FLIGHT_LOG_FIELD_CONDITION_ATTITUDE:
        return is_field_enabled(LOG_SELECT_ATTITUDE);

    case FLIGHT_LOG_FIELD_CONDITION_DEBUG:
        return is_field_enabled(LOG_SELECT_DEBUG) && (_debug_mode != 0);

    case FLIGHT_LOG_FIELD_CONDITION_NEVER:
        return false;

    default:
        return false;
    }
}

/*!
Build condition cache, called from start()
*/
void Blackbox::build_field_condition_cache() // NOLINT(readability-make-member-function-const) false positive
{
    _condition_cache.reset();
    for (size_t ii = FLIGHT_LOG_FIELD_CONDITION_FIRST; ii <= FLIGHT_LOG_FIELD_CONDITION_LAST; ++ii) {
        const auto condition = static_cast<uint8_t>(ii);
        if (test_field_condition_uncached(condition)) {
            _condition_cache.set(condition);
        }
    }
}
