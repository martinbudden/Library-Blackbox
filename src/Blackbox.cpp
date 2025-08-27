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
#include "BlackboxCallbacksBase.h"
#include "BlackboxFieldDefinitions.h"
#include "BlackboxSerialDevice.h"

#include <TimeMicroSeconds.h>
#include <cassert>
#include <cstring>

#ifdef USE_FLASH_TEST_PRBS
void checkFlashStart();
void checkFlashStop();
#endif

enum { BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS = 200 };

/*!
Call during system startup to initialize the 
*/
void Blackbox::init(const config_t& config)
{
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
    _serialDevice.init();

    _config = config;

    _logSelectEnabled = Blackbox::LOG_SELECT_PID
        | Blackbox::LOG_SELECT_SETPOINT
        | Blackbox::LOG_SELECT_RC_COMMANDS
        | Blackbox::LOG_SELECT_GYRO
        | Blackbox::LOG_SELECT_GYRO_UNFILTERED
        | Blackbox::LOG_SELECT_ACCELEROMETER
        | Blackbox::LOG_SELECT_MOTOR
        | Blackbox::LOG_SELECT_MOTOR_RPM;

    resetIterationTimers();

    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    // targetPidLooptimeUs is 1000 for 1kHz loop, 500 for 2kHz loop etc, targetPidLooptimeUs is rounded for short looptimes
    _IInterval = static_cast<int32_t>(32 * 1000 / targetPidLooptimeUs);

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
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
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
    assert(startParameters.motorCount <= blackboxMainState_t::MAX_SUPPORTED_MOTOR_COUNT);
    assert(startParameters.servoCount <= blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT);
    _motorCount = startParameters.motorCount;
    _servoCount = startParameters.servoCount;
    _debugMode = startParameters.debugMode;
    _logSelectEnabled = logSelectEnabled;
    buildFieldConditionCache();
    if (!_serialDevice.open()) {
        setState(STATE_DISABLED);
        return _state;
    }

#if defined(USE_GPS)
    memset(&_gpsState, 0, sizeof(_gpsState));
#endif

    blackboxMainState_t mainState {};
    _callbacks.loadMainState(mainState, 0);
    vbatReference = mainState.vbatLatest;

    //No need to clear the content of _mainStateHistoryRing since our first frame will be an intra which overwrites it

    // We use conditional tests to decide whether or not certain fields should be logged. Since our headers
    // must always agree with the logged data, the results of these tests must not change during logging. So
    // cache those now.
    buildFieldConditionCache();

    //!!blackboxModeActivationConditionPresent = _callbacks.isBlackboxModeActivationConditionPresent();

    resetIterationTimers();

    // Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
    // it finally plays the beep for this arming event.
    blackboxLastArmingBeep = _callbacks.getArmingBeepTimeMicroSeconds();
    blackboxLastFlightModeFlags = _callbacks.rcModeActivationMask(); // record startup status

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

/**
 * Test Motors Blackbox Logging
 */

void Blackbox::startInTestMode()
{
    if (!startedLoggingInTestMode) {
#if false
        if (_config.device == DEVICE_SERIAL) {
            serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
            if (sharedBlackboxAndMspPort) {
                return; // When in test mode, we cannot share the MSP and serial logger port!
            }
        }
#endif
        start();
        startedLoggingInTestMode = true;
    }
}

void Blackbox::stopInTestMode()
{
    if (startedLoggingInTestMode) {
        finish();
        startedLoggingInTestMode = false;
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
    if (!_callbacks.isArmed() && _callbacks.areMotorsRunning()) {
        enum { FIVE_SECONDS_IN_MS = 5000 };
        _resetTime = timeMs() + FIVE_SECONDS_IN_MS;
        return true;
    }
    // Monitor the duration at minimum
    return (timeMs() < _resetTime);
}

void Blackbox::setState(state_e newState)
{
    //Perform initial setup required for the new state
    switch (newState) {
    case STATE_PREPARE_LOG_FILE:
        blackboxLoggedAnyFrames = false;
        break;
    case STATE_SEND_HEADER:
        blackboxHeaderBudget = 0;
        _xmitState.headerIndex = 0;
#if defined(FRAMEWORK_TEST)
        _xmitState.startTime = 0;
#else
        _xmitState.startTime = timeMs();
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
        _slowFrameIterationTimer = _SInterval; //Force a slow frame to be written on the first iteration
#ifdef USE_FLASH_TEST_PRBS
        // Start writing a known pattern as the running state is entered
        checkFlashStart();
#endif
        break;
    case STATE_SHUTTING_DOWN:
        _xmitState.startTime = timeMs();
        break;

#ifdef USE_FLASH_TEST_PRBS
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
    bool shouldWrite = _slowFrameIterationTimer >= _SInterval;

    if (shouldWrite) {
        _callbacks.loadSlowState(_slowState);
    } else {
        blackboxSlowState_t newSlowState {};
        _callbacks.loadSlowState(newSlowState);

        // Only write a slow frame if it was different from the previous state
        if (memcmp(&newSlowState, &_slowState, sizeof(_slowState)) != 0) {
            // Use the new state as our new history
            memcpy(&_slowState, &newSlowState, sizeof(_slowState));
            shouldWrite = true;
        }
    }

    if (shouldWrite) {
        logSFrame();
    }
    return shouldWrite;
}

void Blackbox::resetIterationTimers()
{
    blackboxIteration = 0;
    blackboxLoopIndex = 0;
    _IFrameIndex = 0;
    _PFrameIndex = 0;
    _slowFrameIterationTimer = 0;
}

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
void Blackbox::advanceIterationTimers()
{
    ++_slowFrameIterationTimer;
    ++blackboxIteration;

    if (++blackboxLoopIndex >= _IInterval) {
        blackboxLoopIndex = 0; // value of zero means IFrame will be written on next update
        ++_IFrameIndex; //!! This does not seem to be used anywhere
        _PFrameIndex = 0;
    } else if (++_PFrameIndex >= _PInterval) {
        _PFrameIndex = 0; // value of zero means PFrame will be written on next update, if IFrame not written
    }
}

/*!
Called once every FC loop in order to log the current state
*/
void Blackbox::logIteration(timeUs_t currentTimeUs)
{
    // Write a keyframe every _IInterval frames so we can resynchronise upon missing frames
    if (shouldLogIFrame()) { // ie blackboxLoopIndex == 0
        // Don't log a slow frame if the slow data didn't change (IFrames are already large enough without adding
        // an additional item to write at the same time). Unless we're *only* logging IFrames, then we have no choice.
        if (isOnlyLoggingIFrames()) {
            logSFrameIfNeeded();
        }

        _callbacks.loadMainState(*_mainStateHistory[0], currentTimeUs);
        logIFrame();
    } else {
        logEventArmingBeepIfNeeded();
        logEventFlightModeIfNeeded(); // Check for FlightMode status change event

        if (shouldLogPFrame()) { // ie _PFrameIndex == 0 && _PInterval != 0
            // We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
            // So only log slow frames during loop iterations where we log a main frame.
            logSFrameIfNeeded();

            _callbacks.loadMainState(*_mainStateHistory[0], currentTimeUs);
            logPFrame();
        }
#if defined(USE_GPS)
        if (isFieldEnabled(LOG_SELECT_GPS)) {
            if (shouldLogHFrame()) {
                logHFrame();
                logGFrame(currentTimeUs);
            } else if (_gpsSolutionData.satelliteCount != _gpsState.satelliteCount 
                || _gpsSolutionData.location.latitude != _gpsState.GPS_coord.latitude 
                || _gpsSolutionData.location.longitude != _gpsState.GPS_coord.longitude) {
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
uint32_t Blackbox::update(uint32_t currentTimeUs) // NOLINT(readability-function-cognitive-complexity)
{
    switch (_state) {
    case STATE_STOPPED:
        if (_callbacks.isArmed()) {
            _serialDevice.open();
            start();
        }
#ifdef USE_FLASHFS
        if (IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
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
        blackboxHeaderBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        //On entry of this state, _xmitState.headerIndex is 0 and startTime is initialized
        // Give the UART time to initialize
        //if (timeMs() < _xmitState.startTime + 100) {
        //    break;
        //}
        if (writeHeader() == WRITE_COMPLETE) { // keep on writing chunks of the header until it returns false, signalling completion
            setState(STATE_SEND_MAIN_FIELD_HEADER);
        }
        break;
    case STATE_SEND_MAIN_FIELD_HEADER:
        blackboxHeaderBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        // On entry of this state, _xmitState.headerIndex is 0 and _xmitState.fieldIndex is -1
        if (writeFieldHeaderMain() == WRITE_COMPLETE) { // keep on writing chunks of the main field header until it returns false, signalling completion
#if defined(USE_GPS)
            setState(isFieldEnabled(LOG_SELECT_GPS) ? STATE_SEND_GPS_H_HEADER : STATE_SEND_SLOW_FIELD_HEADER);
#else
            setState(STATE_SEND_SLOW_FIELD_HEADER);
#endif
        }
        break;
#if defined(USE_GPS)
    case STATE_SEND_GPS_H_HEADER:
        blackboxHeaderBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        if (writeFieldHeaderGPS_H() == WRITE_COMPLETE) {
            setState(STATE_SEND_GPS_G_HEADER);
        }
        break;
    case STATE_SEND_GPS_G_HEADER:
        blackboxHeaderBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        if (writeFieldHeaderGPS_G() == WRITE_COMPLETE) {
            setState(STATE_SEND_SLOW_FIELD_HEADER);
        }
        break;
#endif
    case STATE_SEND_SLOW_FIELD_HEADER:
        blackboxHeaderBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
        // On entry of this state, _xmitState.headerIndex is 0 and _xmitState.fieldIndex is -1
        if (writeFieldHeaderSlow() == WRITE_COMPLETE) { // keep on writing chunks of the slow field header until it returns false, signalling completion
            _cacheFlushNextState = STATE_SEND_SYSINFO;
            setState(STATE_CACHE_FLUSH);
        }
        break;
    case STATE_SEND_SYSINFO:
        blackboxHeaderBudget = static_cast<int32_t>(_serialDevice.replenishHeaderBudget());
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
        if (_callbacks.isBlackboxRcModeActive() && shouldLogIFrame()) {
            // Write a log entry so the decoder is aware that our large time/iteration skip is intended
            //flightLogEvent_loggingResume_t resume {
            //    .logIteration = blackboxIteration,
            //    .currentTime = currentTimeUs
            //};
            const log_event_data_u resume = {
                .loggingResume = {
                    .logIteration = blackboxIteration,
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
        // On entry to this state, blackboxIteration, _PFrameIndex and _IFrameIndex are reset to 0
        // Prevent the Pausing of the log on the mode switch if in Motor Test Mode
        if (_callbacks.isBlackboxModeActivationConditionPresent() && !_callbacks.isBlackboxRcModeActive() && !startedLoggingInTestMode) {
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
        if (_serialDevice.endLog(blackboxLoggedAnyFrames) && (timeMs() > _xmitState.startTime + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || _serialDevice.flushForce())) {
            _serialDevice.close();
            setState(STATE_STOPPED);
        }
        break;
#ifdef USE_FLASHFS
    case STATE_START_ERASE:
        blackboxEraseAll();
        setState(STATE_ERASING);
        beeper(BEEPER_BLACKBOX_ERASE);
        break;
    case STATE_ERASING:
        if (isBlackboxErased()) {
            //Done erasing
            setState(STATE_ERASED);
            beeper(BEEPER_BLACKBOX_ERASE);
        }
        break;
    case STATE_ERASED:
        if (!IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
            setState(STATE_STOPPED);
        }
        break;
#endif
    default:
        break;
    }

    // Did we run out of room on the device? Stop!
    if (_serialDevice.isDeviceFull()) {
#ifdef USE_FLASHFS
        if (_state != STATE_ERASING
            && _state != STATE_START_ERASE
            && _state != STATE_ERASED)
#endif
        {
            setState(STATE_STOPPED);
            // ensure we reset the test mode flag if we stop due to full memory card
            startedLoggingInTestMode = false;
        }
    } else { // Only log in test mode if there is room!
        switch (_config.mode) {
        case MODE_MOTOR_TEST:
            // Handle Motor Test Mode
            if (inMotorTestMode()) {
                if (_state==STATE_STOPPED) {
                    startInTestMode();
                }
            } else {
                if (_state!=STATE_STOPPED) {
                    stopInTestMode();
                }
            }
            break;
        case MODE_ALWAYS_ON:
            if (_state==STATE_STOPPED) {
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

static inline uint32_t llog2(uint32_t n) { return 31 - __builtin_clz(n | 1); }  // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)

uint8_t Blackbox::calculateSampleRate(uint16_t pRatio) const
{
    return static_cast<uint8_t>(llog2(32000 / (targetPidLooptimeUs * pRatio)));  // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
}

void Blackbox::logIFrame()
{
    _encoder.beginFrame('I');

    _encoder.writeUnsignedVB(blackboxIteration);

    const blackboxMainState_t* mainState = _mainStateHistory[0];

    _encoder.writeUnsignedVB(mainState->time);

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID)) {
        _encoder.writeSignedVBArray(&mainState->axisPID_P[0], XYZ_AXIS_COUNT);
        _encoder.writeSignedVBArray(&mainState->axisPID_I[0], XYZ_AXIS_COUNT);

        // Don't bother writing the current D term if the corresponding PID setting is zero
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0)) {
            _encoder.writeSignedVB(mainState->axisPID_D[0]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1)) {
            _encoder.writeSignedVB(mainState->axisPID_D[1]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2)) {
            _encoder.writeSignedVB(mainState->axisPID_D[2]);
        }
        _encoder.writeSignedVBArray(&mainState->axisPID_F[0], XYZ_AXIS_COUNT);
    }

    enum { ROLL = 0, PITCH, YAW, THROTTLE };
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS)) {
        // Write roll, pitch and yaw first, these are signed values in the range [-500,500]
        _encoder.writeSigned16VBArray(&mainState->rcCommand[0], 3);

        // Write the throttle separately from the rest of the RC data as it's unsigned.
        // Throttle lies in range [PWM_RANGE_MIN,PWM_RANGE_MAX], ie [1000,2000]
        _encoder.writeUnsignedVB(mainState->rcCommand[THROTTLE]);
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
        _encoder.writeUnsignedVB((vbatReference - mainState->vbatLatest) & LEAST_SIGNIFICANT_14_BITS);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
        // 12bit value directly from ADC
        _encoder.writeSignedVB(mainState->amperageLatest);
    }

#if defined(USE_MAGNETOMETER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER)) {
        _encoder.writeSigned16VBArray(&mainState->magADC[0], XYZ_AXIS_COUNT);
    }
#endif

#if defined(USE_BAROMETER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BAROMETER)) {
        _encoder.writeSignedVB(mainState->baroAlt);
    }
#endif

#if defined(USE_RANGEFINDER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER)) {
        _encoder.writeSignedVB(mainState->surfaceRaw);
    }
#endif

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        _encoder.writeUnsignedVB(mainState->rssi);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO)) {
        _encoder.writeSigned16VBArray(&mainState->gyroADC[0], XYZ_AXIS_COUNT);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED)) {
        _encoder.writeSigned16VBArray(&mainState->gyroUnfiltered[0], XYZ_AXIS_COUNT);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ACC)) {
        _encoder.writeSigned16VBArray(&mainState->accADC[0], XYZ_AXIS_COUNT);
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG)) {
        _encoder.writeSigned16VBArray(&mainState->debug[0], blackboxMainState_t::DEBUG_VALUE_COUNT);
    }

    if (isFieldEnabled(LOG_SELECT_MOTOR)) {
        //Motors can be below minimum output when disarmed, but that doesn't happen much
        _encoder.writeUnsignedVB(mainState->motor[0] - static_cast<int>(_motorOutputLow));

        //Motors tend to be similar to each other so use the first motor's value as a predictor of the others
        for (size_t ii = 1; ii < _motorCount; ++ii) {
            _encoder.writeSignedVB(mainState->motor[ii] - mainState->motor[0]);
        }
    }
#if defined(USE_SERVOS)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SERVOS)) {
        std::array <int32_t, blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT> out;
        for (size_t ii = 0; ii < blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
            out[ii] = mainState->servo[ii] - 1500;
        }
        _encoder.writeTag8_8SVB(&out[0], blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT);
    }
#endif

#if defined(USE_DSHOT_TELEMETRY)
    if (isFieldEnabled(LOG_SELECT_MOTOR_RPM)) {
        for (size_t ii = 0; ii < _motorCount; ++ii) {
            _encoder.writeUnsignedVB(mainState->erpm[ii]);
        }
    }
#endif

    _encoder.endFrame();
// 2=1
// 1=0
// 0=2
    blackboxMainState_t* const history2Save = _mainStateHistory[2];

    //The current state becomes the new "before" state
    _mainStateHistory[1] = _mainStateHistory[0];
    //And since we have no other history, we also use it for the "before, before" state
    _mainStateHistory[2] = _mainStateHistory[0];
    //And advance the current state over to a blank space ready to be filled
    //_mainStateHistory[0] = ((_mainStateHistory[0] - &_mainStateHistoryRing[1]) % 3) + &_mainStateHistoryRing[0];
    _mainStateHistory[0] = history2Save;

    blackboxLoggedAnyFrames = true;
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
    const blackboxMainState_t* mainState = _mainStateHistory[0];
    const blackboxMainState_t* previousMainState = _mainStateHistory[1];

    //No need to store iteration count since its delta is always 1

    // Since the difference between the difference between successive times will be nearly zero (due to consistent
    // looptime spacing), use second-order differences.
    _encoder.writeSignedVB((int32_t) (mainState->time - 2 * _mainStateHistory[1]->time + _mainStateHistory[2]->time));

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID)) {
        //arraySubInt32(&deltas[0], &mainState->axisPID_P[0], &previousMainState->axisPID_P[0], XYZ_AXIS_COUNT);
        std::array<int32_t, XYZ_AXIS_COUNT> deltas = mainState->axisPID_P - previousMainState->axisPID_P;
        _encoder.writeSignedVBArray(&deltas[0], XYZ_AXIS_COUNT);

        // The PID I field changes very slowly, most of the time +-2, so use an encoding
        // that can pack all three fields into one byte in that situation.
        deltas = mainState->axisPID_I - previousMainState->axisPID_I;
        _encoder.writeTag2_3S32(&deltas[0]);

        // The PID D term is frequently set to zero for yaw, which makes the result from the calculation
        // always zero. So don't bother recording D results when PID D terms are zero.
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0)) {
            _encoder.writeSignedVB(mainState->axisPID_D[0] - previousMainState->axisPID_D[0]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_1)) {
            _encoder.writeSignedVB(mainState->axisPID_D[1] - previousMainState->axisPID_D[1]);
        }
        if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_2)) {
            _encoder.writeSignedVB(mainState->axisPID_D[2] - previousMainState->axisPID_D[2]);
        }

        deltas = mainState->axisPID_F - previousMainState->axisPID_F;
        _encoder.writeSignedVBArray(&deltas[0], XYZ_AXIS_COUNT);
    }

    //std::array<int32_t, 4> setpointDeltas;
    // RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
    // can pack multiple values per byte:
    //for (size_t ii = 0; ii < 4; ++ii) {
    //    deltas[ii] = mainState->rcCommand[ii] - previousMainState->rcCommand[ii];
    //    setpointDeltas[ii] = mainState->setpoint[ii] - previousMainState->setpoint[ii];
    //}

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS)) {
        const std::array<int32_t, 4> deltas = mainState->rcCommand - previousMainState->rcCommand;
        _encoder.writeTag8_4S16(&deltas[0]);
    }
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT)) {
        const std::array<int32_t, 4> deltas = mainState->setpoint - previousMainState->setpoint;
        _encoder.writeTag8_4S16(&deltas[0]);
    }

    enum { MAX_DELTA_COUNT = 8 };
    std::array<int32_t, MAX_DELTA_COUNT> deltas;
    //Check for sensors that are updated periodically (so deltas are normally zero)
    int optionalFieldCount = 0;

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE)) {
        deltas[optionalFieldCount++] = mainState->vbatLatest - previousMainState->vbatLatest;
    }

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC)) {
        deltas[optionalFieldCount++] = mainState->amperageLatest - previousMainState->amperageLatest;
    }

#if defined(USE_MAGNETOMETER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            deltas[optionalFieldCount++] = mainState->magADC[ii] - previousMainState->magADC[ii];
        }
    }
#endif

#if defined(USE_BAROMETER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BAROMETER)) {
        deltas[optionalFieldCount++] = mainState->baroAlt - previousMainState->baroAlt;
    }
#endif

#if defined(USE_RANGEFINDER)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER)) {
        deltas[optionalFieldCount++] = mainState->surfaceRaw - previousMainState->surfaceRaw;
    }
#endif

    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        deltas[optionalFieldCount++] = mainState->rssi - previousMainState->rssi;
    }

    _encoder.writeTag8_8SVB(&deltas[0], optionalFieldCount);

    //Since gyros, accelerometers and motors are noisy, base their predictions on the average of the history:
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO)) {
        //writeMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC), XYZ_AXIS_COUNT);
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->gyroADC[ii] + _mainStateHistory[2]->gyroADC[ii]) / 2;
            _encoder.writeSignedVB(mainState->gyroADC[ii] - predictor);
        }
    }
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED)) {
        //writeMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroUnfilt), XYZ_AXIS_COUNT);
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->gyroUnfiltered[ii] + _mainStateHistory[2]->gyroUnfiltered[ii]) / 2;
            _encoder.writeSignedVB(mainState->gyroUnfiltered[ii] - predictor);
        }
    }
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ACC)) {
        //writeMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADC), XYZ_AXIS_COUNT);
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->accADC[ii] + _mainStateHistory[2]->accADC[ii]) / 2;
            _encoder.writeSignedVB(mainState->accADC[ii] - predictor);
        }
    }
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG)) {
        //writeMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, debug), blackboxMainState_t::DEBUG_VALUE_COUNT);
        for (size_t ii = 0; ii < blackboxMainState_t::DEBUG_VALUE_COUNT; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->debug[ii] + _mainStateHistory[2]->debug[ii]) / 2;
            _encoder.writeSignedVB(mainState->debug[ii] - predictor);
        }
    }

    if (isFieldEnabled(LOG_SELECT_MOTOR)) {
        //writeMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor), _motorCount);
        for (size_t ii = 0; ii < _motorCount; ++ii) {
            const int32_t predictor = (_mainStateHistory[1]->motor[ii] + _mainStateHistory[2]->motor[ii]) / 2;
            _encoder.writeSignedVB(mainState->motor[ii] - predictor);
        }
    }

#if defined(USE_SERVOS)
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SERVOS)) {
        std::array <int32_t, blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT> out;
        for (size_t ii = 0; ii < blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
            out[ii] = mainState->servo[ii] - 1500;
        }
        _encoder.writeTag8_8SVB(&out[0], blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT);
    }
#endif

#if defined(USE_DSHOT_TELEMETRY)
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
    blackboxMainState_t* const history2Save = _mainStateHistory[2];
    _mainStateHistory[2] = _mainStateHistory[1];
    _mainStateHistory[1] = _mainStateHistory[0];
    //_mainStateHistory[0] = &_mainStateHistoryRing[0] + ((_mainStateHistory[0] - &_mainStateHistoryRing[1]) % 3);
    _mainStateHistory[0] = history2Save;

    blackboxLoggedAnyFrames = true;
    _encoder.endFrame();
}

/* Write the contents of the global "_slowState" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
void Blackbox::logSFrame()
{
    _encoder.beginFrame('S');

    _encoder.writeUnsignedVB(_slowState.flightModeFlags);
    _encoder.writeUnsignedVB(_slowState.stateFlags);

    // Most of the time these three values will be able to pack into one byte for us:
    const std::array<int32_t, 3> values {
        _slowState.failsafePhase,
        _slowState.rxSignalReceived ? 1 : 0,
        _slowState.rxFlightChannelsValid ? 1 : 0
    };

    _encoder.writeTag2_3S32(&values[0]);

    _slowFrameIterationTimer = 0;

    _encoder.endFrame();
}

#if defined(USE_GPS)
/*
 * If the GPS home point has been updated, or every 128 I-frames (~10 seconds), write the
 * GPS home position.
 *
 * We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
 * still be interpreted correctly.
 */
bool Blackbox::shouldLogHFrame() const
{
    if ((_gpsHomeLocation.latitude != _gpsState.home.latitude
         || _gpsHomeLocation.longitude != _gpsState.home.longitude
         || (_PFrameIndex == _IInterval / 2 && _IFrameIndex % 128 == 0)) // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
        && isFieldEnabled(LOG_SELECT_GPS)) {
        return true; // NOLINT(readability-simplify-boolean-expr)
    }
    return false;
}

void Blackbox::logHFrame()
{
    _encoder.beginFrame('H');

    _encoder.writeSignedVB(_gpsHomeLocation.latitude);
    _encoder.writeSignedVB(_gpsHomeLocation.longitude);
     //log home altitude, in increments of 0.1m
    _encoder.writeSignedVB(_gpsHomeLocation.altitudeCm / 10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
    // Suggestion: it'd be great if we could grab the GPS current time and write that too

    _gpsState.home = _gpsHomeLocation;

    _encoder.endFrame();
}

void Blackbox::logGFrame(timeUs_t currentTimeUs)
{
    _encoder.beginFrame('G');

    /*
     * If we're logging every frame, then a GPS frame always appears just after a frame with the
     * currentTime timestamp in the log, so the reader can just use that timestamp for the GPS frame.
     *
     * If we're not logging every frame, we need to store the time of this GPS frame.
     */
    if (testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME)) {
        // Predict the time of the last frame in the main log
        _encoder.writeUnsignedVB(currentTimeUs - _mainStateHistory[1]->time);
    }

    _encoder.writeUnsignedVB(_gpsSolutionData.satelliteCount);
    _encoder.writeSignedVB(_gpsSolutionData.location.latitude - _gpsState.home.latitude);
    _encoder.writeSignedVB(_gpsSolutionData.location.longitude - _gpsState.home.longitude);
    // log altitude in increments of 0.1m 
    _encoder.writeSignedVB(_gpsSolutionData.location.altitudeCm / 10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)

    //if (gpsConfig()->gps_use_3d_speed) {
    //    _encoder.writeUnsignedVB(_gpsSolutionData.speed3d);
    //} else {
        _encoder.writeUnsignedVB(_gpsSolutionData.groundSpeed);
    //}

    _encoder.writeUnsignedVB(_gpsSolutionData.groundCourse);

    _gpsState.satelliteCount = _gpsSolutionData.satelliteCount;
    _gpsState.GPS_coord = _gpsSolutionData.location;

    _encoder.endFrame();
}
#endif // USE_GPS


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
            enum { LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG = 128 };
            _encoder.write(data->inflightAdjustment.adjustmentFunction + LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
            _encoder.writeFloat(data->inflightAdjustment.newFloatValue);
        } else {
            _encoder.write(data->inflightAdjustment.adjustmentFunction);
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
    const uint32_t armingBeepTimeMicroSeconds = _callbacks.getArmingBeepTimeMicroSeconds();
    if (armingBeepTimeMicroSeconds != blackboxLastArmingBeep) {
        blackboxLastArmingBeep = armingBeepTimeMicroSeconds;
        const log_event_data_u eventData {
            .syncBeep = {
                .time  = blackboxLastArmingBeep
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
    if (memcmp(&_callbacks._rcModeActivationMask, &blackboxLastFlightModeFlags, sizeof(blackboxLastFlightModeFlags))) {
        static flightLogEvent_flightMode_t eventData {}; // Add new data for current flight mode flags
        eventData.lastFlags = blackboxLastFlightModeFlags;
        memcpy(&blackboxLastFlightModeFlags, &_callbacks._rcModeActivationMask, sizeof(blackboxLastFlightModeFlags));
        memcpy(&eventData.flags, &_callbacks._rcModeActivationMask, sizeof(eventData.flags));
        logEvent(LOG_EVENT_FLIGHTMODE, reinterpret_cast<flightLogEventData_u*>(&eventData)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    }
#endif
    const uint32_t rcModeActivationMask = _callbacks.rcModeActivationMask();
    if (rcModeActivationMask != blackboxLastFlightModeFlags) {
        const log_event_data_u eventData = {
            .flightMode = {
                .flags = rcModeActivationMask,
                .lastFlags = blackboxLastFlightModeFlags
            }
        };
        blackboxLastFlightModeFlags = rcModeActivationMask;
        logEvent(LOG_EVENT_FLIGHTMODE, &eventData);
    }
}
