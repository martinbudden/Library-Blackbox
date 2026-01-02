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

#include "BlackboxFieldDefinitions.h"


#pragma pack(push, 1)
// simple fields
struct blackbox_simple_field_definition_t {
    const char *name;
    int8_t fieldNameIndex;
    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
};

// conditional fields, used for GPS
struct blackbox_conditional_field_definition_t {
    const char *name;
    int8_t fieldNameIndex;
    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
    uint8_t condition;
};

// delta fields, used for P-frames
struct blackbox_delta_field_definition_t {
    const char* name;
    int8_t fieldNameIndex;
    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    flight_log_field_condition_e condition; // Decide whether this field should appear in the log
};
#pragma pack(pop)

enum flight_log_fieldSign_e {
    FLIGHT_LOG_FIELD_UNSIGNED = 0,
    FLIGHT_LOG_FIELD_SIGNED   = 1
};

enum flight_logField_predictor_e {
    // No prediction:
    FLIGHT_LOG_FIELD_PREDICTOR_0              = 0,

    // Predict that the field is the same as last frame:
    FLIGHT_LOG_FIELD_PREDICTOR_PREVIOUS       = 1,

    // Predict that the slope between this field and the previous item is the same as that between the past two history items:
    FLIGHT_LOG_FIELD_PREDICTOR_STRAIGHT_LINE  = 2,

    // Predict that this field is the same as the average of the last two history items:
    FLIGHT_LOG_FIELD_PREDICTOR_AVERAGE_2      = 3,

    // Predict that this field is minthrottle
    FLIGHT_LOG_FIELD_PREDICTOR_MINTHROTTLE    = 4,

    // Predict that this field is the same as motor 0
    FLIGHT_LOG_FIELD_PREDICTOR_MOTOR_0        = 5,

    // This field always increments
    FLIGHT_LOG_FIELD_PREDICTOR_INC            = 6,

    // Predict this GPS co-ordinate is the GPS home co-ordinate (or no prediction if that coordinate is not set)
    FLIGHT_LOG_FIELD_PREDICTOR_HOME_COORD     = 7,

    // Predict 1500
    FLIGHT_LOG_FIELD_PREDICTOR_1500           = 8,

    // Predict vbatref, the reference ADC level stored in the header
    FLIGHT_LOG_FIELD_PREDICTOR_VBATREF        = 9,

    // Predict the last time value written in the main stream
    FLIGHT_LOG_FIELD_PREDICTOR_LAST_MAIN_FRAME_TIME = 10,

    // Predict that this field is the minimum motor output
    FLIGHT_LOG_FIELD_PREDICTOR_MINMOTOR       = 11
};

enum FlightLogFieldEncoding_e {
    FLIGHT_LOG_FIELD_ENCODING_SIGNED_VB       = 0, // Signed variable-byte
    FLIGHT_LOG_FIELD_ENCODING_UNSIGNED_VB     = 1, // Unsigned variable-byte
    FLIGHT_LOG_FIELD_ENCODING_NEG_14BIT       = 3, // Unsigned variable-byte but we negate the value before storing, value is 14 bits, used for battery voltage
    FLIGHT_LOG_FIELD_ENCODING_TAG8_8SVB       = 6,
    FLIGHT_LOG_FIELD_ENCODING_TAG2_3S32       = 7,
    FLIGHT_LOG_FIELD_ENCODING_TAG8_4S16       = 8,
    FLIGHT_LOG_FIELD_ENCODING_NULL            = 9, // Nothing is written to the file, take value to be zero
    FLIGHT_LOG_FIELD_ENCODING_TAG2_3SVARIABLE = 10 // Seems this is not used
};
