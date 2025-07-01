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

#include "Blackbox.h"
#include "BlackboxHeader.h"
#include "BlackboxSerialDevice.h"
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
#define CONCAT2(_1,_2) CONCAT(_1, _2)
#define CONCAT3(_1,_2,_3)  CONCAT(CONCAT(_1, _2), _3)
#define CONCAT4(_1,_2,_3,_4)  CONCAT(CONCAT3(_1, _2, _3), _4)

#define UNSIGNED FLIGHT_LOG_FIELD_UNSIGNED
#define SIGNED FLIGHT_LOG_FIELD_SIGNED
#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
// NOLINTEND(cppcoreguidelines-macro-usage)

// Rarely-updated fields
enum { SLOW_FIELD_COUNT = 5 };
static const std::array<blackboxSimpleFieldDefinition_t, SLOW_FIELD_COUNT> blackboxSlowFields = {{
    {"flightModeFlags",       -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},
    {"stateFlags",            -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},

    {"failsafePhase",         -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxSignalReceived",      -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)}
}};

#if defined(USE_GPS)
// GPS position/velocity frame
enum { GPS_G_FIELD_COUNT = 7 };
static const std::array<blackboxConditionalFieldDefinition_t, GPS_G_FIELD_COUNT> blackboxGpsGFields = {{
    {"time",              -1, UNSIGNED, PREDICT(LAST_MAIN_FRAME_TIME), ENCODING(UNSIGNED_VB), CONDITION(NOT_LOGGING_EVERY_FRAME)},
    {"GPS_numSat",        -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_coord",          0, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_coord",          1, SIGNED,   PREDICT(HOME_COORD), ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_altitude",      -1, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB),   CONDITION(ALWAYS)},
    {"GPS_speed",         -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)},
    {"GPS_ground_course", -1, UNSIGNED, PREDICT(0),          ENCODING(UNSIGNED_VB), CONDITION(ALWAYS)}
}};

// GPS home frame
enum { GPS_H_FIELD_COUNT = 3 };
static const std::array<blackboxSimpleFieldDefinition_t, GPS_H_FIELD_COUNT> blackboxGpsHFields = {{
    {"GPS_home",           0, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)},
    {"GPS_home",           1, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)},
    {"GPS_home",           2, SIGNED,   PREDICT(0),          ENCODING(SIGNED_VB)}
}};
#endif

/**
 * Description of the blackbox fields we are writing in our main I (intra) and P (inter) frames. 
 * This description is written into the flight log header so the log can be properly interpreted.
 * These definitions don't actually cause the encoding to happen,
 * we have to encode the flight log ourselves in logPFrame and logIFrame in a way that matches the encoding we've promised here).
 */
//static const std::array<Blackbox::blackboxDeltaFieldDefinition_t, MAIN_FIELD_COUNT> blackboxMainFields = {{
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
    /* loopIteration doesn't appear in P frames since it always increments */
    {"loopIteration",-1, UNSIGNED, .Ipredict = PREDICT(0),     .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC),           .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    // Time advances pretty steadily so the P-frame prediction is a straight line
    {"time",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisP",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisP",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    // I terms get special packed encoding in P frames:
    {"axisI",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisI",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisI",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisD",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_0)},
    {"axisD",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_1)},
    {"axisD",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_2)},
    {"axisF",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisF",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisF",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    // rcCommands are encoded together as a group in P-frames:
    {"rcCommand",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   3, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    // setpoint - define 4 fields like rcCommand to use the same encoding. setpoint[4] contains the mixer throttle
    {"setpoint",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},

    {"vbatLatest",    -1, UNSIGNED, .Ipredict = PREDICT(VBATREF),  .Iencode = ENCODING(NEG_14BIT),   .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), CONDITION(BATTERY_VOLTAGE)},
    {"amperageLatest",-1, SIGNED,   .Ipredict = PREDICT(0),        .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), CONDITION(AMPERAGE_ADC)},
#if defined(USE_MAGNETOMETER)
    {"magADC",      0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAGNETOMETER)},
    {"magADC",      1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAGNETOMETER)},
    {"magADC",      2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAGNETOMETER)},
#endif
#if defined(USE_BAROMETER)
    {"baroAlt",    -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(BAROMETER)},
#endif
#if defined(USE_RANGEFINDER)
    {"surfaceRaw",   -1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(RANGEFINDER)},
#endif
    {"rssi",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(RSSI)},
    // Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact
    {"gyroADC",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADC",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADC",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroUnfilt",  0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO_UNFILTERED)},
    {"gyroUnfilt",  1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO_UNFILTERED)},
    {"gyroUnfilt",  2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO_UNFILTERED)},
    {"accSmooth",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accSmooth",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accSmooth",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"debug",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG)},
    {"debug",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG)},
    {"debug",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG)},
    {"debug",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG)},
    {"debug",       4, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG)},
    {"debug",       5, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG)},
    {"debug",       6, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG)},
    {"debug",       7, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG)},
    // Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones):
    // must match with enum { MAX_SUPPORTED_MOTORS = 4 };
    {"motor",       0, UNSIGNED, .Ipredict = PREDICT(MINMOTOR), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
    /* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
    {"motor",       1, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor",       2, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor",       3, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
#if defined(USE_EIGHT_MOTORS)
    {"motor",       4, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor",       5, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor",       6, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor",       7, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},
#endif
#if defined(USE_SERVOS)
    // must match with MAX_SUPPORTED_SERVO_COUNT
    // NOTE (ledvinap, hwarhurst): Decoding would fail if previous encoding is also TAG8_8SVB and does not have exactly 8 values. To fix it, inserting ENCODING_NULL dummy value should force end of previous group.
    {"servo",       0, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(TAG8_8SVB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(SERVOS)},
    {"servo",       1, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(TAG8_8SVB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(SERVOS)},
    {"servo",       2, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(TAG8_8SVB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(SERVOS)},
    {"servo",       3, UNSIGNED, .Ipredict = PREDICT(1500),    .Iencode = ENCODING(TAG8_8SVB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(SERVOS)},
#endif
#if defined(USE_DSHOT_TELEMETRY)
    // must match with MAX_SUPPORTED_MOTOR_COUNT
    // eRPM / 100
    {"eRPM",        0, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(MOTOR_1_HAS_RPM)},
    {"eRPM",        1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(MOTOR_2_HAS_RPM)},
    {"eRPM",        2, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(MOTOR_3_HAS_RPM)},
    {"eRPM",        3, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(MOTOR_4_HAS_RPM)},
#if defined(USE_EIGHT_MOTORS)
    {"eRPM",        4, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(MOTOR_5_HAS_RPM)},
    {"eRPM",        5, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(MOTOR_6_HAS_RPM)},
    {"eRPM",        6, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(MOTOR_7_HAS_RPM)},
    {"eRPM",        7, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(MOTOR_8_HAS_RPM)},
#endif
#endif // USE_DSHOT_TELEMETRY
#undef CONDITION
};

int Blackbox::printfv(const char* fmt, va_list va)
{
    return tfp_format(&_serialDevice, BlackboxEncoder::putc, fmt, va);
}


//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
int Blackbox::printf(const char* fmt, ...) // NOLINT(cert-dcl50-cpp)
{
    va_list va;
    va_start(va, fmt);
    const int written = printfv(fmt, va);
    va_end(va);

    return written;
}

/*!
`printf` a Blackbox header line with a leading "H " and trailing "\n" added automatically.
`blackboxHeaderBudget` is decreased to account for the number of bytes written.
 */
size_t Blackbox::headerPrintfHeaderLine(const char* name, const char* fmt, ...) // NOLINT(cert-dcl50-cpp)
{
    headerWrite('H');
    headerWrite(' ');
    headerWriteString(name);
    headerWrite(':');

    va_list va;
    va_start(va, fmt);

    const int written = printfv(fmt, va);

    va_end(va);

    headerWrite('\n');

    blackboxHeaderBudget -= written + 3;

    return written + 3;
}

size_t Blackbox::headerPrintf(const char *fmt, ...) // NOLINT(cert-dcl50-cpp)
{
    va_list va;
    va_start(va, fmt);

    const int ret = printfv(fmt, va); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)

    va_end(va);

    return ret;
}

void Blackbox::headerWrite(uint8_t value)
{
    _encoder.write(value);
}

size_t Blackbox::headerWriteString(const char* s)
{
    const auto* pos = reinterpret_cast<const uint8_t*>(s); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    size_t length = 0;
    while (*pos) {
        headerWrite(*pos);
        ++pos; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ++length;
        //if (length >= _buf.max_size()) {
        //    break;
        //}
    }

    return length;
}

bool Blackbox::headerReserveBufferSpace()
{
    enum { LONGEST_LINE_LENGTH = 64 };
    if (_serialDevice.reserveBufferSpace(LONGEST_LINE_LENGTH) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        return false; // NOLINT(readability-simplify-boolean-expr)
    }
    return true;
}


Blackbox::write_e Blackbox::writeHeader()
{
    // Transmit the header in chunks so we don't overflow its transmit
    // buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
    if (_serialDevice.reserveBufferSpace(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        for (int ii = 0; ii < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[_xmitState.headerIndex] != '\0'; ++ii) {
            //Write header, ie
            //"H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
            //"H Data version:2\n"
            _encoder.write(blackboxHeader[_xmitState.headerIndex]);
            blackboxHeaderBudget--;
            ++_xmitState.headerIndex;
        }
        if (blackboxHeader[_xmitState.headerIndex] == 0) {
            return WRITE_COMPLETE; // we have finished
        }
    }
    return WRITE_NOT_COMPLETE; //  we have more to write
}

Blackbox::write_e Blackbox::writeFieldHeaderMain() // NOLINT(readability-function-cognitive-complexity)
{
    // We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
    // the whole header.

    // On our first call we need to print the name of the header and a colon
    const int32_t fieldCount = sizeof(blackboxMainFields) / sizeof(blackboxDeltaFieldDefinition_t);
    if (_xmitState.fieldIndex == -1) {
        const uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[_xmitState.headerIndex]);
        if (_serialDevice.reserveBufferSpace(charsToBeWritten) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
            return WRITE_NOT_COMPLETE; // Try again later
        }
        if (_xmitState.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT) {
            blackboxHeaderBudget -= headerPrintf("H Field P %s:", blackboxFieldHeaderNames[_xmitState.headerIndex]); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        } else {
            blackboxHeaderBudget -= headerPrintf("H Field I %s:", blackboxFieldHeaderNames[_xmitState.headerIndex]); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        }
        ++_xmitState.fieldIndex;
    }
    if (_xmitState.headerIndex == 0) {
        //0: H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisD[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],vbatLatest,amperageLatest,gyroADC[0],gyroADC[1],gyroADC[2],motor[0],motor[1],motor[2],motor[3]
        for (; _xmitState.fieldIndex < fieldCount; ++_xmitState.fieldIndex) {
            const blackboxDeltaFieldDefinition_t& def = blackboxMainFields[_xmitState.fieldIndex];
            if (testFieldCondition(def.condition)) {
                if (def.fieldNameIndex == -1) {
                    headerPrintf(_xmitState.fieldIndex == 0 ? "%s" : ",%s", def.name); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
                } else {
                    headerPrintf(_xmitState.fieldIndex == 0 ? "%s[%d]" : ",%s[%d]", def.name, def.fieldNameIndex); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
                }
            }
        }
    } else {
        //1: H Field I signed:   0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1, 0,0,0,0
        //2: H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,9,0,0,0,0,11,5,5,5
        //3: H Field I encoding: 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,3,1,0,0,0, 1,0,0,0
        //4: H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3, 3,3,3,3
        //5: H Field P encoding: 9,0,0,0,0,7,7,7,0,0,0,8,8,8,8,6,6,0,0,0, 0,0,0,0
        for (; _xmitState.fieldIndex < fieldCount; ++_xmitState.fieldIndex) {
            const blackboxDeltaFieldDefinition_t& def = blackboxMainFields[_xmitState.fieldIndex];
            const uint8_t value = 
                _xmitState.headerIndex == 1 ? def.isSigned : 
                _xmitState.headerIndex == 2 ? def.Ipredict : 
                _xmitState.headerIndex == 3 ? def.Iencode : 
                _xmitState.headerIndex == 4 ? def.Ppredict : def.Pencode;
            if (testFieldCondition(def.condition)) {
                headerPrintf(_xmitState.fieldIndex == 0 ? "%d" : ",%d", value); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
            }
        }
    }
    if (_xmitState.fieldIndex == fieldCount && _serialDevice.reserveBufferSpace(1) == BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        --blackboxHeaderBudget;
        headerWrite('\n');
        ++_xmitState.headerIndex;
        _xmitState.fieldIndex = -1; // set fieldIndex to -1 to write the field header next time round
    }

    // return WRITE_COMPLETE if we have nothing more to write
    return _xmitState.headerIndex < BLACKBOX_DELTA_FIELD_HEADER_COUNT ? WRITE_NOT_COMPLETE : WRITE_COMPLETE;
}

Blackbox::write_e Blackbox::writeFieldHeaderSlow() // NOLINT(readability-function-cognitive-complexity)
{
    // We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
    // the whole header.

    const int32_t fieldCount = SLOW_FIELD_COUNT;
    // On our first call we need to print the name of the header and a colon
    if (_xmitState.fieldIndex == -1) {
        const uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[_xmitState.headerIndex]);
        if (_serialDevice.reserveBufferSpace(charsToBeWritten) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
            return WRITE_NOT_COMPLETE; // Try again later
        }
        blackboxHeaderBudget -= headerPrintf("H Field S %s:", blackboxFieldHeaderNames[_xmitState.headerIndex]); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        ++_xmitState.fieldIndex;
    }
    if (_xmitState.headerIndex == 0) {
        //0:H Field S name:flightModeFlags,stateFlags,failsafePhase,rxSignalReceived,rxFlightChannelsValid
        for (; _xmitState.fieldIndex < fieldCount; ++_xmitState.fieldIndex) {
            const blackboxSimpleFieldDefinition_t& def = blackboxSlowFields[_xmitState.fieldIndex];
            headerPrintf(_xmitState.fieldIndex == 0 ? "%s" : ",%s", def.name); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        }
    } else {
        //1:H Field S signed:   0,0,0,0,0
        //2:H Field S predictor:0,0,0,0,0
        //3:H Field S encoding: 1,1,7,7,7
        for (; _xmitState.fieldIndex < SLOW_FIELD_COUNT; ++_xmitState.fieldIndex) {
            const blackboxSimpleFieldDefinition_t& def = blackboxSlowFields[_xmitState.fieldIndex];
            const uint8_t value = 
                _xmitState.headerIndex == 1 ? def.isSigned :
                _xmitState.headerIndex == 2 ? def.predict : def.encode;
            headerPrintf(_xmitState.fieldIndex == 0 ? "%d" : ",%d", value); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
        }
    }
    if (_xmitState.fieldIndex == fieldCount && _serialDevice.reserveBufferSpace(1) == BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        --blackboxHeaderBudget;
        headerWrite('\n');
        ++_xmitState.headerIndex;
        _xmitState.fieldIndex = -1; // set fieldIndex to -1 to write the field header next time round
    }
    // return WRITE_COMPLETE if we have nothing more to write
    return _xmitState.headerIndex < BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? WRITE_NOT_COMPLETE : WRITE_COMPLETE;
}

Blackbox::write_e Blackbox::writeFieldHeaderGPS_H() // NOLINT(readability-convert-member-functions-to-static)
{
    return WRITE_COMPLETE;
}

Blackbox::write_e Blackbox::writeFieldHeaderGPS_G() // NOLINT(readability-convert-member-functions-to-static)
{
    return WRITE_COMPLETE;
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-type-union-access,readability-magic-numbers)

/*!
Called from buildFieldConditionCache(), which is called from start()
*/
bool Blackbox::testFieldConditionUncached(flight_log_field_condition_e condition) const
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
        return isFieldEnabled(LOG_SELECT_MOTOR) && (static_cast<int32_t>(_motorCount) > condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1);

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
        return isFieldEnabled(LOG_SELECT_MOTOR_RPM) && (static_cast<int32_t>(_motorCount) > condition - FLIGHT_LOG_FIELD_CONDITION_MOTOR_1_HAS_RPM);

    case FLIGHT_LOG_FIELD_CONDITION_SERVOS:
        return isFieldEnabled(LOG_SELECT_SERVO) && (_servoCount > 0);

    case FLIGHT_LOG_FIELD_CONDITION_PID:
        return isFieldEnabled(LOG_SELECT_PID);

    case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1:
        [[fallthrough]];
    case FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2: {
        // always log the dterm, even if it is zero
        // not logging dterm saves little space and means if it is tuned during flight, the value won't be logged.
        //const uint8_t currentPidProfileDTerm = _callbacks.getCurrentPidProfileDTermConstant(condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0);
        return  isFieldEnabled(LOG_SELECT_PID); // && (currentPidProfileDTerm != 0);
    }
    case FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS:
        return isFieldEnabled(LOG_SELECT_RC_COMMANDS);

    case FLIGHT_LOG_FIELD_CONDITION_SETPOINT:
        return isFieldEnabled(LOG_SELECT_SETPOINT);

    case FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER:
        return isFieldEnabled(LOG_SELECT_MAGNETOMETER);

    case FLIGHT_LOG_FIELD_CONDITION_BAROMETER:
        return isFieldEnabled(LOG_SELECT_BAROMETER);

    case FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE:
        return isFieldEnabled(LOG_SELECT_BATTERY_VOLTMETER);

    case FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC:
        return isFieldEnabled(LOG_SELECT_CURRENT_METER);

    case FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER:
        return isFieldEnabled(LOG_SELECT_RANGEFINDER);

    case FLIGHT_LOG_FIELD_CONDITION_RSSI:
        return isFieldEnabled(LOG_SELECT_RSSI);

    case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
        return blackboxPInterval != blackboxIInterval;

    case FLIGHT_LOG_FIELD_CONDITION_GYRO:
        return isFieldEnabled(LOG_SELECT_GYRO);

    case FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED:
        return isFieldEnabled(LOG_SELECT_GYRO_UNFILTERED);

    case FLIGHT_LOG_FIELD_CONDITION_ACC:
        return isFieldEnabled(LOG_SELECT_ACCELEROMETER);

    case FLIGHT_LOG_FIELD_CONDITION_DEBUG:
        return isFieldEnabled(LOG_SELECT_DEBUG) && (_debugMode != 0);

    case FLIGHT_LOG_FIELD_CONDITION_NEVER:
        return false;

    default:
        return false;
    }
}

/*!
Build condition cache, called from start()
*/
void Blackbox::buildFieldConditionCache() // NOLINT(readability-make-member-function-const) false positive
{
    _conditionCache.reset();
    for (size_t ii = FLIGHT_LOG_FIELD_CONDITION_FIRST; ii <= FLIGHT_LOG_FIELD_CONDITION_LAST; ++ii) {
        const auto condition = static_cast<flight_log_field_condition_e>(ii);
        if (testFieldConditionUncached(condition)) {
            _conditionCache.set(condition);
        }
    }
}
