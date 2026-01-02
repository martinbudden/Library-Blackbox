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

#pragma pack(push, 1)
// We pack this struct so that padding doesn't interfere with memcmp()
// This data is updated really infrequently:
struct blackbox_slow_state_t {
    uint32_t flightModeFlags; // extend this data size (from uint16_t)
    uint8_t stateFlags;
    uint8_t failsafePhase;
    bool rxSignalReceived; // sizeof(bool) is commonly 1, but not defined by the C++ standard
    bool rxFlightChannelsValid;
};
#pragma pack(pop)


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

    uint32_t time;
    int32_t baroAlt;
    int32_t surfaceRaw;
    int32_t amperageLatest;
    uint16_t vbatLatest;
    uint16_t rssi;

    std::array<int32_t, RPY_AXIS_COUNT> axisPID_P;
    std::array<int32_t, RPY_AXIS_COUNT> axisPID_I;
    std::array<int32_t, RPY_AXIS_COUNT> axisPID_D;
    std::array<int32_t, RPY_AXIS_COUNT> axisPID_S;
    std::array<int32_t, RPY_AXIS_COUNT> axisPID_K;

    std::array<int16_t, 4> rcCommand;
    std::array<int16_t, 4> setpoint;

    std::array<int16_t, XYZ_AXIS_COUNT> gyroADC;
    std::array<int16_t, XYZ_AXIS_COUNT> gyroUnfiltered;
    std::array<int16_t, XYZ_AXIS_COUNT> accADC;
    std::array<int16_t, XYZ_AXIS_COUNT> orientation; // only x,y,z are stored; w is always positive
    std::array<int16_t, XYZ_AXIS_COUNT> magADC;

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
    virtual void loadSlowState(blackbox_slow_state_t& slowState) = 0;

    //! Load the main state of the blackbox, used for I-frames and P-frames
    virtual void loadMainState(blackbox_main_state_t& mainState, uint32_t currentTimeUs) = 0;

    virtual bool isArmed() const = 0;
    virtual bool isBlackboxModeActive() const = 0;
    virtual bool isBlackboxEraseModeActive() const = 0;
    virtual bool isBlackboxModeActivationConditionPresent() const = 0;
    virtual uint32_t getArmingBeepTimeMicroseconds() const = 0;
    virtual bool areMotorsRunning() const = 0;
    virtual uint32_t rcModeActivationMask() const = 0; // lower 32 bits of BOX_COUNT bits
};
