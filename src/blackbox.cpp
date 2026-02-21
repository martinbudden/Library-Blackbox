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
#include "blackbox_callbacks_base.h"
#include "blackbox_field_definitions.h"
#include "blackbox_serial_device.h"

#include <cassert>
#include <cstring>
#include <time_microseconds.h>

#if defined(USE_FLASH_TEST_PRBS)
void checkFlashStart();
void checkFlashStop();
#endif

enum { BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS = 200 };

/*!
Call during system startup.
*/
void Blackbox::init(const config_t& config)
{
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _serialDevice.init();

    _config = config;

    _logSelectEnabled =
        Blackbox::LOG_SELECT_PID
        | Blackbox::LOG_SELECT_PID_KTERM
        | Blackbox::LOG_SELECT_PID_DTERM_ROLL
        | Blackbox::LOG_SELECT_PID_DTERM_PITCH
        //| Blackbox::LOG_SELECT_PID_STERM_ROLL
        //| Blackbox::LOG_SELECT_PID_STERM_PITCH
        //| Blackbox::LOG_SELECT_PID_STERM_YAW
        | Blackbox::LOG_SELECT_SETPOINT
        | Blackbox::LOG_SELECT_RC_COMMANDS
        | Blackbox::LOG_SELECT_GYRO
        | Blackbox::LOG_SELECT_GYRO_UNFILTERED
        | Blackbox::LOG_SELECT_ACCELEROMETER
        //| Blackbox::LOG_SELECT_ATTITUDE
        | Blackbox::LOG_SELECT_MOTOR
        | Blackbox::LOG_SELECT_MOTOR_RPM;

    resetIterationTimers();

    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    // _targetPidLooptime_us is 1000 for 1kHz loop, 500 for 2kHz loop etc, _targetPidLooptime_us is rounded for short looptimes
    _IInterval = static_cast<int32_t>(32 * 1000 / _targetPidLooptime_us);

    _PInterval = static_cast<int32_t>(1U << _config.sample_rate);
    if (_PInterval > _IInterval) {
        _PInterval = 0; // log only I frames if logging frequency is too low
    }

    // S-frame is written every 256*32 = 8192ms, approx every 8 seconds
    _SInterval = _IInterval * 256;

    if (_config.device == DEVICE_NONE) {
        setState(STATE_DISABLED);
    } else if (_config.mode == MODE_ALWAYS_ON) {
        start();
    } else {
        setState(STATE_STOPPED);
    }
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

Blackbox::state_e Blackbox::start()
{
    return start({.debugMode = _debugMode, .motorCount = static_cast<uint8_t>(_motorCount), .servoCount = static_cast<uint8_t>(_servoCount)});
}

Blackbox::state_e Blackbox::start(const start_t& startParameters)
{
    return start(startParameters, _logSelectEnabled);
}

/*!
Start Blackbox logging if it is not already running. Intended to be called upon arming.
*/
Blackbox::state_e Blackbox::start(const start_t& startParameters, uint32_t logSelectEnabled)
{
    assert(startParameters.motorCount <= blackbox_main_state_t::MAX_SUPPORTED_MOTOR_COUNT);
    assert(startParameters.servoCount <= blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT);
    _motorCount = startParameters.motorCount;
    _servoCount = startParameters.servoCount;
    _debugMode = startParameters.debugMode;
    _logSelectEnabled = logSelectEnabled;

    // We use conditional tests to decide whether or not certain fields should be logged. Since our headers
    // must always agree with the logged data, the results of these tests must not change during logging.
    // So cache those now.
    buildFieldConditionCache();
    if (!_serialDevice.open()) {
        setState(STATE_DISABLED);
        return _state;
    }

#if defined(LIBRARY_BLACKBOX_USE_GPS)
    memset(&_gpsState, 0, sizeof(_gpsState));
#endif

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE)) {
        // If we are logging battery voltage, then load_main_state to get the reference battery voltage.
        blackbox_main_state_t mainState {};
        _callbacks.load_main_state(mainState, 0);
        _vbatReference = mainState.vbat_latest;
    }

    // No need to clear the content of _mainStateHistoryRing since our first frame will be an intra which overwrites it

    //!!blackboxModeActivationConditionPresent = _callbacks.is_blackbox_mode_activation_condition_present();

    resetIterationTimers();

    // Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
    // it finally plays the beep for this arming event.
    _lastArmingBeep = _callbacks.get_arming_beep_time_microseconds();
    _lastFlightModeFlags = _callbacks.rc_mode_activation_mask(); // record startup status

    setState(STATE_PREPARE_LOG_FILE);
    return _state;
}

/*!
Begin Blackbox shutdown.
*/
void Blackbox::finish()
{
    switch (_state) {
    case STATE_DISABLED:
        [[fallthrough]];
    case STATE_STOPPED:
        [[fallthrough]];
    case STATE_SHUTTING_DOWN:
        // We're already stopped/shutting down
        break;
    case STATE_RUNNING:
        [[fallthrough]];
    case STATE_PAUSED:
        logEvent(LOG_EVENT_LOG_END, nullptr);
        [[fallthrough]];
    default:
        setState(STATE_SHUTTING_DOWN);
    }
}

void Blackbox::endLog()
{
    _serialDevice.endLog(true);
}

/*
Test Motors Blackbox Logging
*/

void Blackbox::replenishHeaderBudget()
{
    _headerBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
}

void Blackbox::startInTestMode()
{
    if (!_startedLoggingInTestMode) {
#if false
        if (_config.device == DEVICE_SERIAL) {
            serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
            if (sharedBlackboxAndMspPort) {
                return; // When in test mode, we cannot share the MSP and serial logger port!
            }
        }
#endif
        start();
        _startedLoggingInTestMode = true;
    }
}

void Blackbox::stopInTestMode()
{
    if (_startedLoggingInTestMode) {
        finish();
        _startedLoggingInTestMode = false;
    }
}

/**
 * We are going to monitor the MSP_SET_MOTOR target variables motor_disarmed[] for values other than minthrottle
 * on reading a value (i.e. the user is testing the motors), then we enable test mode logging;
 * we monitor when the values return to minthrottle and start a delay timer (5 seconds); if
 * the test motors are left at minimum throttle for this delay timer, then we assume we are done testing and
 * shutdown the logger.
 *
 * Of course, after the 5 seconds and shutdown of the logger, the system will be re-enabled to allow the
 * test mode to trigger again; its just that the data will be in a second, third, fourth etc log file.
 */
bool Blackbox::inMotorTestMode()
{
    if (!_callbacks.is_armed() && _callbacks.are_motors_running()) {
        enum { FIVE_SECONDS_IN_MS = 5000 };
        _resetTime = time_ms() + FIVE_SECONDS_IN_MS;
        return true;
    }
    // Monitor the duration at minimum
    return (time_ms() < _resetTime);
}

void Blackbox::setState(state_e newState)
{
    //Perform initial setup required for the new state
    switch (newState) {
    case STATE_PREPARE_LOG_FILE:
        _loggedAnyFrames = false;
        break;
    case STATE_SEND_HEADER:
        _headerBudget = 0;
        _xmitState.headerIndex = 0;
#if defined(FRAMEWORK_TEST)
        _xmitState.startTime = 0;
#else
        _xmitState.startTime = time_ms();
#endif
        break;
    case STATE_SEND_MAIN_FIELD_HEADER:
        [[fallthrough]];
    case STATE_SEND_GPS_G_HEADER:
        [[fallthrough]];
    case STATE_SEND_GPS_H_HEADER:
        [[fallthrough]];
    case STATE_SEND_SLOW_FIELD_HEADER:
        _xmitState.headerIndex = 0;
        _xmitState.fieldIndex = -1;
        break;
    case STATE_SEND_SYSINFO:
        _xmitState.headerIndex = 0;
        break;
    case STATE_RUNNING:
        _SFrameIndex = _SInterval; //Force a slow frame to be written on the first iteration
#if defined(USE_FLASH_TEST_PRBS)
        // Start writing a known pattern as the running state is entered
        checkFlashStart();
#endif
        break;
    case STATE_SHUTTING_DOWN:
        _xmitState.startTime = time_ms();
        break;

#if defined(USE_FLASH_TEST_PRBS)
    case STATE_STOPPED:
        // Now that the log is shut down, verify it
        checkFlashStop();
        break;
#endif
    default:
        break;
    }
    _state = newState;
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than _SInterval logging iterations
 * since the field was last logged.
 */
bool Blackbox::logSFrameIfNeeded()
{
    // Write the slow frame periodically so it can be recovered if we ever lose sync
    if (_SFrameIndex >= _SInterval) {
        _callbacks.load_slow_state(_slowState);
        logSFrame();
        return true;
    }

    // Only write a slow frame if it was different from the previous state
    blackbox_slow_state_t newSlowState {};
    _callbacks.load_slow_state(newSlowState);
    if (memcmp(&newSlowState, &_slowState, sizeof(_slowState)) != 0) {
        // Use the new state as our new history
        memcpy(&_slowState, &newSlowState, sizeof(_slowState));
        logSFrame();
        return true;
    }
    return false;
}

void Blackbox::resetIterationTimers()
{
    _iteration = 0;
    _loopIndex = 0;
    _IFrameIndex = 0;
    _PFrameIndex = 0;
    _SFrameIndex = 0;
}

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
void Blackbox::advanceIterationTimers()
{
    ++_SFrameIndex;
    ++_iteration;

    if (++_loopIndex >= _IInterval) {
        _loopIndex = 0; // value of zero means IFrame will be written on next update
        ++_IFrameIndex;
        _PFrameIndex = 0;
    } else if (++_PFrameIndex >= _PInterval) {
        _PFrameIndex = 0; // value of zero means PFrame will be written on next update, if IFrame not written
    }
}

/*!
Called once every FC loop in order to log the current state
*/
void Blackbox::logIteration(time_us_t currentTimeUs)
{
    // Write a keyframe every _IInterval frames so we can resynchronise upon missing frames
    if (shouldLogIFrame()) { // ie _loopIndex == 0
        // Don't log a slow frame if the slow data didn't change (IFrames are already large enough without adding
        // an additional item to write at the same time). Unless we're *only* logging IFrames, then we have no choice.
        if (isOnlyLoggingIFrames()) {
            logSFrameIfNeeded();
        }

        _callbacks.load_main_state(*_mainStateHistory[0], currentTimeUs);
        logIFrame();
    } else {
        logEventArmingBeepIfNeeded();
        logEventFlightModeIfNeeded(); // Check for FlightMode status change event

        if (shouldLogPFrame()) { // ie _PFrameIndex == 0 && _PInterval != 0
            // We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
            // So only log slow frames during loop iterations where we log a main frame.
            logSFrameIfNeeded();

            _callbacks.load_main_state(*_mainStateHistory[0], currentTimeUs);
            logPFrame();
        }
#if defined(LIBRARY_BLACKBOX_USE_GPS)
        if (isFieldEnabled(LOG_SELECT_GPS)) {
            blackbox_gps_state_t gpsStateNew {};
            _callbacks.load_gps_state(gpsStateNew);

            const bool gpsStateChanged =
                gpsStateNew.satellite_count != _gpsState.satellite_count
                || gpsStateNew.latitude_degrees1E7 != _gpsState.latitude_degrees1E7
                || gpsStateNew.longitude_degrees1E7 != _gpsState.longitude_degrees1E7;

            _gpsState = gpsStateNew;

            if (shouldLogHFrame()) {
                _gpsHomeLocation.latitude_degrees1E7 = _gpsState.home_latitude_degrees1E7;
                _gpsHomeLocation.longitude_degrees1E7 = _gpsState.home_longitude_degrees1E7;
                _gpsHomeLocation.altitude_cm = _gpsState.home_altitude_cm;
                logHFrame();
                logGFrame(currentTimeUs);
            } else if (gpsStateChanged) {
                //We could check for velocity changes as well but I doubt it changes independent of position
                logGFrame(currentTimeUs);
            }
        }
#endif
    }

    //Flush every iteration so that our runtime variance is minimized
    _serialDevice.flush();
}

/*!
Called each flight loop iteration to perform blackbox logging.
*/
uint32_t Blackbox::update_log(uint32_t currentTimeUs) // NOLINT(readability-function-cognitive-complexity)
{
    switch (_state) {
    case STATE_STOPPED:
        if (_callbacks.is_armed()) {
            _serialDevice.open();
            start();
        }
#if defined(BLACKBOX_LIBRARY_USE_FLASHFS)
        if (_callbacks.is_blackbox_erase_mode_active()) {
            setState(STATE_START_ERASE);
        }
#endif
        break;
    case STATE_PREPARE_LOG_FILE:
        if (_serialDevice.beginLog()) {
            setState(STATE_SEND_HEADER);
        }
        break;
    case STATE_SEND_HEADER:
        _headerBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        //On entry of this state, _xmitState.headerIndex is 0 and startTime is initialized
        // Give the UART time to initialize
        //if (time_ms() < _xmitState.startTime + 100) {
        //    break;
        //}
        if (writeHeader() == WRITE_COMPLETE) { // keep on writing chunks of the header until it returns false, signalling completion
            setState(STATE_SEND_MAIN_FIELD_HEADER);
        }
        break;
    case STATE_SEND_MAIN_FIELD_HEADER:
        _headerBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        // On entry of this state, _xmitState.headerIndex is 0 and _xmitState.fieldIndex is -1
        if (writeFieldHeaderMain() == WRITE_COMPLETE) { // keep on writing chunks of the main field header until it returns false, signalling completion
#if defined(LIBRARY_BLACKBOX_USE_GPS)
            setState(isFieldEnabled(LOG_SELECT_GPS) ? STATE_SEND_GPS_H_HEADER : STATE_SEND_SLOW_FIELD_HEADER);
#else
            setState(STATE_SEND_SLOW_FIELD_HEADER);
#endif
        }
        break;
#if defined(LIBRARY_BLACKBOX_USE_GPS)
    case STATE_SEND_GPS_H_HEADER:
        _headerBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        if (writeFieldHeaderGPS_H() == WRITE_COMPLETE) {
            setState(STATE_SEND_GPS_G_HEADER);
        }
        break;
    case STATE_SEND_GPS_G_HEADER:
        _headerBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        if (writeFieldHeaderGPS_G() == WRITE_COMPLETE) {
            setState(STATE_SEND_SLOW_FIELD_HEADER);
        }
        break;
#endif
    case STATE_SEND_SLOW_FIELD_HEADER:
        _headerBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        // On entry of this state, _xmitState.headerIndex is 0 and _xmitState.fieldIndex is -1
        if (writeFieldHeaderSlow() == WRITE_COMPLETE) { // keep on writing chunks of the slow field header until it returns false, signalling completion
            _cacheFlushNextState = STATE_SEND_SYSINFO;
            setState(STATE_CACHE_FLUSH);
        }
        break;
    case STATE_SEND_SYSINFO:
        _headerBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        //On entry of this state, _xmitState.headerIndex is 0

        //Keep writing chunks of the system info headers until it returns true to signal completion
        if (writeSystemInformation() == WRITE_COMPLETE) {
            /*
             * Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
             * (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
             * could wipe out the end of the header if we weren't careful)
             */
            _cacheFlushNextState = STATE_RUNNING;
            setState(STATE_CACHE_FLUSH);
        }
        break;
    case STATE_CACHE_FLUSH:
        // Flush the cache and wait until all possible entries have been written to the media
        if (_serialDevice.flushForceComplete()) {
            setState(_cacheFlushNextState);
        }
        break;
    case STATE_PAUSED:
        // Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
        if (_callbacks.is_blackbox_mode_active() && shouldLogIFrame()) {
            // Write a log entry so the decoder is aware that our large time/iteration skip is intended
            //flightLogEvent_loggingResume_t resume {
            //    .logIteration = _iteration,
            //    .currentTime = currentTimeUs
            //};
            const log_event_data_u resume = {
                .loggingResume = {
                    .logIteration = _iteration,
                    .currentTime = currentTimeUs
                }
            };
            logEvent(LOG_EVENT_LOGGING_RESUME, &resume);
            setState(STATE_RUNNING);

            logIteration(currentTimeUs);
        }
        // Keep the logging timers ticking so our log iteration continues to advance
        advanceIterationTimers();
        break;
    case STATE_RUNNING:
        // On entry to this state, _iteration, _PFrameIndex and _IFrameIndex are reset to 0
        // Prevent the Pausing of the log on the mode switch if in Motor Test Mode
        if (_callbacks.is_blackbox_mode_activation_condition_present() && !_callbacks.is_blackbox_mode_active() && !_startedLoggingInTestMode) {
            setState(STATE_PAUSED);
        } else {
            logIteration(currentTimeUs);
        }
        advanceIterationTimers();
        break;
    case STATE_SHUTTING_DOWN:
        //On entry of this state, startTime is set
        /*
         * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
         * since releasing the port clears the Tx buffer.
         *
         * Don't wait longer than it could possibly take if something funky happens.
         */
        if (_serialDevice.endLog(_loggedAnyFrames) && (time_ms() > _xmitState.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || _serialDevice.flushForce())) { // cppcheck-suppress unsignedLessThanZero
            _serialDevice.close();
            setState(STATE_STOPPED);
        }
        break;
#if defined(BLACKBOX_LIBRARY_USE_FLASHFS)
    case STATE_START_ERASE:
        _serialDevice.eraseAll();
        setState(STATE_ERASING);
        _callbacks.beep();
        break;
    case STATE_ERASING:
        if (_serialDevice.isErased()) {
            //Done erasing
            setState(STATE_ERASED);
            _callbacks.beep();
        }
        break;
    case STATE_ERASED:
        if (!_callbacks.is_blackbox_erase_mode_active()) {
            setState(STATE_STOPPED);
        }
        break;
#endif
    default:
        break;
    }

    // Did we run out of room on the device? Stop!
    if (_serialDevice.isDeviceFull()) {
#if defined(BLACKBOX_LIBRARY_USE_FLASHFS)
        if (_state != STATE_ERASING && _state != STATE_START_ERASE && _state != STATE_ERASED)
#endif
        {
            setState(STATE_STOPPED);
            // ensure we reset the test mode flag if we stop due to full memory card
            _startedLoggingInTestMode = false;
        }
    } else { // Only log in test mode if there is room!
        switch (_config.mode) {
        case MODE_MOTOR_TEST:
            // Handle Motor Test Mode
            if (inMotorTestMode()) {
                if (_state == STATE_STOPPED) {
                    startInTestMode();
                }
            } else {
                if (_state != STATE_STOPPED) {
                    stopInTestMode();
                }
            }
            break;
        case MODE_ALWAYS_ON:
            if (_state == STATE_STOPPED) {
                startInTestMode();
            }
            break;
        case MODE_NORMAL:
            [[fallthrough]];
        default:
            break;
        }
    }
    return _state;
}

static inline uint32_t llog2(uint32_t n) { return static_cast<uint32_t>(31 - __builtin_clz(n | 1)); }  // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)

uint8_t Blackbox::calculateSampleRate(uint16_t pRatio) const
{
    return static_cast<uint8_t>(llog2(32000 / (_targetPidLooptime_us * pRatio)));  // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
}

/*!
Return true if it is safe to edit the Blackbox configuration.
*/
bool Blackbox::mayEditConfig()
{
    return _state <= STATE_STOPPED;
}

void Blackbox::logIFrame() // NOLINT(readability-function-cognitive-complexity)
{
    _encoder.beginFrame('I');

    _encoder.writeUnsignedVB(_iteration);

    const blackbox_main_state_t* mainState = _mainStateHistory[0];

    _encoder.writeUnsignedVB(mainState->time_us);

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID)) {
        _encoder.writeSignedVBArray(&mainState->axis_pid_p[0], RPY_AXIS_COUNT);
        _encoder.writeSignedVBArray(&mainState->axis_pid_i[0], RPY_AXIS_COUNT);

        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_ROLL)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[0]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_PITCH)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[1]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_YAW)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[2]);
        }

        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_K)) {
            _encoder.writeSignedVBArray(&mainState->axis_pid_k[0], RPY_AXIS_COUNT);
        }

        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_ROLL)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[0]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_PITCH)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[1]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_YAW)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[2]);
        }
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS)) {
        // Write roll, pitch and yaw first, these are signed values in the range [-500,500]
        _encoder.writeSigned16VBArray(&mainState->rc_command[0], 3);

        // Write the throttle separately from the rest of the RC data as it's unsigned.
        // Throttle lies in range [PWM_RANGE_MIN,PWM_RANGE_MAX], ie [1000,2000]
        enum { ROLL = 0, PITCH, YAW, THROTTLE };
        _encoder.writeUnsignedVB(static_cast<uint32_t>(mainState->rc_command[THROTTLE]));
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT)) {
        // Write setpoint roll, pitch, yaw, and throttle
        _encoder.writeSigned16VBArray(&mainState->setpoint[0], 4);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE)) {
        //Our voltage is expected to decrease over the course of the flight, so store our difference from
        //the reference:
        // Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
        enum { LEAST_SIGNIFICANT_14_BITS = 0x3FFF };
        _encoder.writeUnsignedVB((_vbatReference - mainState->vbat_latest) & LEAST_SIGNIFICANT_14_BITS);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_CURRENT)) {
        // 12bit value directly from ADC
        _encoder.writeSignedVB(mainState->amperage_latest);
    }

#if defined(LIBRARY_BLACKBOX_USE_MAGNETOMETER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER)) {
        _encoder.writeSigned16VBArray(&mainState->mag_adc[0], XYZ_AXIS_COUNT);
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_BAROMETER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BAROMETER)) {
        _encoder.writeSignedVB(mainState->baro_altitude);
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_RANGEFINDER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER)) {
        _encoder.writeSignedVB(mainState->surface_raw);
    }
#endif

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        _encoder.writeUnsignedVB(mainState->rssi);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO)) {
        _encoder.writeSigned16VBArray(&mainState->gyro_adc[0], XYZ_AXIS_COUNT);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED)) {
        _encoder.writeSigned16VBArray(&mainState->gyro_unfiltered[0], XYZ_AXIS_COUNT);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ACC)) {
        _encoder.writeSigned16VBArray(&mainState->acc_adc[0], XYZ_AXIS_COUNT);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ATTITUDE)) {
        _encoder.writeSigned16VBArray(&mainState->orientation[0], XYZ_AXIS_COUNT);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG)) {
        _encoder.writeSigned16VBArray(&mainState->debug[0], blackbox_main_state_t::DEBUG_VALUE_COUNT);
    }

    if (isFieldEnabled(LOG_SELECT_MOTOR)) {
        //Motors can be below minimum output when disarmed, but that doesn't happen much
        _encoder.writeUnsignedVB(static_cast<uint32_t>(mainState->motor[0] - static_cast<int>(_motorOutputLow)));

        //Motors tend to be similar to each other so use the first motor's value as a predictor of the others
        for (size_t ii = 1; ii < _motorCount; ++ii) {
            _encoder.writeSignedVB(mainState->motor[ii] - mainState->motor[0]);
        }
    }
#if defined(LIBRARY_BLACKBOX_USE_SERVOS)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SERVOS)) {
        std::array <int32_t, blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT> out;
        for (size_t ii = 0; ii < blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
            out[ii] = mainState->servo[ii] - 1500; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        }
        _encoder.writeTag8_8SVB(&out[0], blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT);
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_DSHOT_TELEMETRY)
    if (isFieldEnabled(LOG_SELECT_MOTOR_RPM)) {
        for (size_t ii = 0; ii < _motorCount; ++ii) {
            _encoder.writeUnsignedVB(static_cast<uint32_t>(mainState->erpm[ii]));
        }
    }
#endif

    _encoder.endFrame();
// 2=1
// 1=0
// 0=2
    blackbox_main_state_t* const history2Save = _mainStateHistory[2];

    // The current state becomes the new "before" state
    _mainStateHistory[1] = _mainStateHistory[0];
    // And since we have no other history, we also use it for the "before, before" state
    _mainStateHistory[2] = _mainStateHistory[0];
    // And advance the current state over to a blank space ready to be filled
    // _mainStateHistory[0] = ((_mainStateHistory[0] - &_mainStateHistoryRing[1]) % 3) + &_mainStateHistoryRing[0];
    _mainStateHistory[0] = history2Save;

    _loggedAnyFrames = true;
}

inline std::array<int32_t, 3> operator-(const std::array<int32_t, 3>& a, const std::array<int32_t, 3>& b) // NOLINT(fuchsia-overloaded-operator)
{
    return std::array<int32_t, 3> {
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2]
    };
}

inline std::array<int32_t, 4> operator-(const std::array<int16_t, 4>& a, const std::array<int16_t, 4>& b) // NOLINT(fuchsia-overloaded-operator)
{
    return std::array<int32_t, 4> {
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
        a[3] - b[3]
    };
}

void Blackbox::logPFrame() // NOLINT(readability-function-cognitive-complexity)
{
    _encoder.beginFrame('P');
    const blackbox_main_state_t* mainState = _mainStateHistory[0];
    const blackbox_main_state_t* previousMainState = _mainStateHistory[1];

    //No need to store iteration count since its delta is always 1

    // Since the difference between the difference between successive times will be nearly zero (due to consistent
    // looptime spacing), use second-order differences.
    _encoder.writeSignedVB(static_cast<int32_t>(mainState->time_us - 2 * _mainStateHistory[1]->time_us + _mainStateHistory[2]->time_us));

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID)) {
        //arraySubInt32(&deltas[0], &mainState->axis_pid_p[0], &previousMainState->axis_pid_p[0], RPY_AXIS_COUNT);
        std::array<int32_t, RPY_AXIS_COUNT> deltas = mainState->axis_pid_p - previousMainState->axis_pid_p;
        _encoder.writeSignedVBArray(&deltas[0], RPY_AXIS_COUNT);

        // The PID I field changes very slowly, most of the time +-2, so use an encoding
        // that can pack all three fields into one byte in that situation.
        deltas = mainState->axis_pid_i - previousMainState->axis_pid_i;
        _encoder.writeTag2_3S32(&deltas[0]);

        // The PID D term is frequently set to zero for yaw, which makes the result from the calculation
        // always zero. So don't bother recording D results when PID D terms are zero.
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_ROLL)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[0] - previousMainState->axis_pid_d[0]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_PITCH)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[1] - previousMainState->axis_pid_d[1]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_YAW)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[2] - previousMainState->axis_pid_d[2]);
        }

        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_K)) {
            deltas = mainState->axis_pid_k - previousMainState->axis_pid_k;
            _encoder.writeSignedVBArray(&deltas[0], RPY_AXIS_COUNT);
        }

        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_ROLL)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[0] - previousMainState->axis_pid_s[0]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_PITCH)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[1] - previousMainState->axis_pid_s[1]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_YAW)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[2] - previousMainState->axis_pid_s[2]);
        }
    }

    // RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS)) {
        const std::array<int32_t, 4> deltas = mainState->rc_command - previousMainState->rc_command;
        _encoder.writeTag8_4S16(&deltas[0]);
    }
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT)) {
        const std::array<int32_t, 4> deltas = mainState->setpoint - previousMainState->setpoint;
        _encoder.writeTag8_4S16(&deltas[0]);
    }

    enum { MAX_DELTA_COUNT = 8 };
    std::array<int32_t, MAX_DELTA_COUNT> deltas;
    //Check for sensors that are updated periodically (so deltas are normally zero)
    size_t optionalFieldCount = 0;

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE)) {
        deltas[optionalFieldCount++] = mainState->vbat_latest - previousMainState->vbat_latest;
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_CURRENT)) {
        deltas[optionalFieldCount++] = mainState->amperage_latest - previousMainState->amperage_latest;
    }

#if defined(LIBRARY_BLACKBOX_USE_MAGNETOMETER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            deltas[optionalFieldCount++] = mainState->mag_adc[ii] - previousMainState->mag_adc[ii];
        }
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_BAROMETER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BAROMETER)) {
        deltas[optionalFieldCount++] = mainState->baro_altitude - previousMainState->baro_altitude;
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_RANGEFINDER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER)) {
        deltas[optionalFieldCount++] = mainState->surface_raw - previousMainState->surface_raw;
    }
#endif

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        deltas[optionalFieldCount++] = mainState->rssi - previousMainState->rssi;
    }

    _encoder.writeTag8_8SVB(&deltas[0], optionalFieldCount);

    //Since gyros, accelerometers and motors are noisy, base their predictions on the average of the history:
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->gyro_adc[ii] + _mainStateHistory[2]->gyro_adc[ii]) / 2;
            _encoder.writeSignedVB(mainState->gyro_adc[ii] - predictor);
        }
    }
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->gyro_unfiltered[ii] + _mainStateHistory[2]->gyro_unfiltered[ii]) / 2;
            _encoder.writeSignedVB(mainState->gyro_unfiltered[ii] - predictor);
        }
    }
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ACC)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->acc_adc[ii] + _mainStateHistory[2]->acc_adc[ii]) / 2;
            _encoder.writeSignedVB(mainState->acc_adc[ii] - predictor);
        }
    }
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ATTITUDE)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->orientation[ii] + _mainStateHistory[2]->orientation[ii]) / 2;
            _encoder.writeSignedVB(mainState->orientation[ii] - predictor);
        }
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG)) {
        for (size_t ii = 0; ii < blackbox_main_state_t::DEBUG_VALUE_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->debug[ii] + _mainStateHistory[2]->debug[ii]) / 2;
            _encoder.writeSignedVB(mainState->debug[ii] - predictor);
        }
    }

    if (isFieldEnabled(LOG_SELECT_MOTOR)) {
        for (size_t ii = 0; ii < _motorCount; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->motor[ii] + _mainStateHistory[2]->motor[ii]) / 2;
            _encoder.writeSignedVB(mainState->motor[ii] - predictor);
        }
    }

#if defined(LIBRARY_BLACKBOX_USE_SERVOS)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SERVOS)) {
        std::array <int32_t, blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT> out;
        for (size_t ii = 0; ii < blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
            out[ii] = mainState->servo[ii] - 1500; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        }
        _encoder.writeTag8_8SVB(&out[0], blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT);
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_DSHOT_TELEMETRY)
    if (isFieldEnabled(LOG_SELECT_MOTOR_RPM)) {
        for (size_t ii = 0; ii < _motorCount; ++ii) {
            _encoder.writeSignedVB(mainState->erpm[ii] - previousMainState->erpm[ii]);
        }
    }
#endif
    //Rotate our history buffers
// 2=1
// 1=0
// 0=2
    blackbox_main_state_t* const history2Save = _mainStateHistory[2];
    _mainStateHistory[2] = _mainStateHistory[1];
    _mainStateHistory[1] = _mainStateHistory[0];
    //_mainStateHistory[0] = &_mainStateHistoryRing[0] + ((_mainStateHistory[0] - &_mainStateHistoryRing[1]) % 3);
    _mainStateHistory[0] = history2Save;

    _loggedAnyFrames = true;
    _encoder.endFrame();
}

/*!
Write the contents of the global "_slowState" to the log as an S frame.
Because this data is logged so infrequently, delta updates are not reasonable, so we log independent frames.
*/
void Blackbox::logSFrame()
{
    _encoder.beginFrame('S');

    _encoder.writeUnsignedVB(_slowState.flight_mode_flags);
    _encoder.writeUnsignedVB(_slowState.state_flags);

    // Most of the time these three values will be able to pack into one byte.
    const std::array<int32_t, 3> values {
        _slowState.failsafe_phase,
        _slowState.rx_signal_received ? 1 : 0,
        _slowState.rx_flight_channe_is_valid ? 1 : 0
    };

    _encoder.writeTag2_3S32(&values[0]);

    _SFrameIndex = 0;

    _encoder.endFrame();
}

#if defined(LIBRARY_BLACKBOX_USE_GPS)
/*!
If the GPS home point has been updated, or every 128 I-frames (~10 seconds), write the
GPS home position.

We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
still be interpreted correctly.
*/
bool Blackbox::shouldLogHFrame() const
{
    if ((_gpsHomeLocation.latitude_degrees1E7 != _gpsState.home_latitude_degrees1E7
         || _gpsHomeLocation.longitude_degrees1E7 != _gpsState.home_longitude_degrees1E7
         || (_PFrameIndex == _IInterval / 2 && _IFrameIndex % 128 == 0)) // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
        && isFieldEnabled(LOG_SELECT_GPS)) {
        return true; // NOLINT(readability-simplify-boolean-expr)
    }
    return false;
}

void Blackbox::logHFrame()
{
    _encoder.beginFrame('H');

    _encoder.writeSignedVB(_gpsState.home_latitude_degrees1E7);
    _encoder.writeSignedVB(_gpsState.home_longitude_degrees1E7);
    _encoder.writeSignedVB(_gpsState.home_altitude_cm / 10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)

    _encoder.endFrame();
}

void Blackbox::logGFrame(time_us_t currentTimeUs)
{
    _encoder.beginFrame('G');

    // If we're logging every frame, then a GPS frame always appears just after a frame with the
    // currentTime timestamp in the log, so the reader can just use that timestamp for the GPS frame.
    // If we're not logging every frame, we need to store the time of this GPS frame.
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME)) {
        // Predict the time of the last frame in the main log
        _encoder.writeUnsignedVB(currentTimeUs - _mainStateHistory[1]->time_us);
    }

    _encoder.writeUnsignedVB(_gpsState.satellite_count);
    _encoder.writeSignedVB(_gpsState.latitude_degrees1E7 - _gpsHomeLocation.latitude_degrees1E7);
    _encoder.writeSignedVB(_gpsState.longitude_degrees1E7 - _gpsHomeLocation.longitude_degrees1E7);
    // log altitude in increments of 0.1m
    _encoder.writeSignedVB(_gpsState.altitude_cm / 10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)

    if (_config.gps_use_3d_speed) {
        _encoder.writeUnsignedVB(static_cast<uint32_t>(_gpsState.speed3d_cmps));
    } else {
        _encoder.writeUnsignedVB(static_cast<uint32_t>(_gpsState.ground_speed_cmps));
    }

    _encoder.writeUnsignedVB(static_cast<uint32_t>(_gpsState.ground_course_deci_degrees));

    _encoder.writeSignedVB(_gpsState.velocity_north_cmps);
    _encoder.writeSignedVB(_gpsState.velocity_east_cmps);
    _encoder.writeSignedVB(_gpsState.velocity_down_cmps);

    _encoder.endFrame();
}
#endif // LIBRARY_BLACKBOX_USE_GPS


/*!
Write the given event to the log immediately
*/
bool Blackbox::logEvent(log_event_e event, const log_event_data_u* data)
{
    // Only allow events to be logged after headers have been written
    //if (!(_state == STATE_RUNNING || _state == STATE_PAUSED)) {
    if ((_state != STATE_RUNNING) && (_state != STATE_PAUSED)) {
        return false;
    }

    //Shared header for event frames
    _encoder.beginFrame('E');
    _encoder.write(event);

    //Now serialize the data for this specific frame type
    switch (event) {
    case LOG_EVENT_SYNC_BEEP:
        _encoder.writeUnsignedVB(data->syncBeep.time);
        break;
    case LOG_EVENT_FLIGHTMODE: // New flightmode flags write
        _encoder.writeUnsignedVB(data->flightMode.flags);
        _encoder.writeUnsignedVB(data->flightMode.lastFlags);
        break;
    case LOG_EVENT_DISARM:
        _encoder.writeUnsignedVB(data->disarm.reason);
        break;
    case LOG_EVENT_INFLIGHT_ADJUSTMENT:
        if (data->inflightAdjustment.floatFlag) {
            enum { LOG_EVENT_INFLIGHT_ADJUSTMENT_FLOAT_VALUE_FLAG = 128 };
            _encoder.write(data->inflightAdjustment.adjustment + LOG_EVENT_INFLIGHT_ADJUSTMENT_FLOAT_VALUE_FLAG);
            _encoder.writeFloat(data->inflightAdjustment.newFloatValue);
        } else {
            _encoder.write(data->inflightAdjustment.adjustment);
            _encoder.writeSignedVB(data->inflightAdjustment.newValue);
        }
        break;
    case LOG_EVENT_LOGGING_RESUME:
        _encoder.writeUnsignedVB(data->loggingResume.logIteration);
        _encoder.writeUnsignedVB(data->loggingResume.currentTime);
        break;
    case LOG_EVENT_LOG_END:
        // data is nullptr for LOG_EVENT_LOG_END
        headerWriteString("End of log");
        _encoder.write(0);
        break;
    default:
        break;
    }

    _encoder.endFrame();

    return true;
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
void Blackbox::logEventArmingBeepIfNeeded()
{
    // Use != so that we can still detect a change if the counter wraps
    const uint32_t armingBeepTimeMicroseconds = _callbacks.get_arming_beep_time_microseconds();
    if (armingBeepTimeMicroseconds != _lastArmingBeep) {
        _lastArmingBeep = armingBeepTimeMicroseconds;
        const log_event_data_u eventData {
            .syncBeep = {
                .time  = _lastArmingBeep
            }
        };
        logEvent(LOG_EVENT_SYNC_BEEP, &eventData);
    }
}

/* monitor the flight mode event status and trigger an event record if the state changes */
void Blackbox::logEventFlightModeIfNeeded()
{
    // Use != so that we can still detect a change if the counter wraps
#if false
    if (memcmp(&_callbacks._rc_mode_activation_mask, &_lastFlightModeFlags, sizeof(_lastFlightModeFlags))) {
        static flightLogEvent_flightMode_t eventData {}; // Add new data for current flight mode flags
        eventData.lastFlags = _lastFlightModeFlags;
        memcpy(&_lastFlightModeFlags, &_callbacks._rc_mode_activation_mask, sizeof(_lastFlightModeFlags));
        memcpy(&eventData.flags, &_callbacks._rc_mode_activation_mask, sizeof(eventData.flags));
        logEvent(LOG_EVENT_FLIGHTMODE, reinterpret_cast<flightLogEventData_u*>(&eventData)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    }
#endif
    const uint32_t rc_mode_activation_mask = _callbacks.rc_mode_activation_mask();
    if (rc_mode_activation_mask != _lastFlightModeFlags) {
        const log_event_data_u eventData = {
            .flightMode = {
                .flags = rc_mode_activation_mask,
                .lastFlags = _lastFlightModeFlags
            }
        };
        _lastFlightModeFlags = rc_mode_activation_mask;
        logEvent(LOG_EVENT_FLIGHTMODE, &eventData);
    }
}
