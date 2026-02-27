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
    _serial_device.init();

    _config = config;

    _log_select_enabled =
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

    reset_iteration_timers();

    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    // _target_pid_looptime_us is 1000 for 1kHz loop, 500 for 2kHz loop etc, _target_pid_looptime_us is rounded for short looptimes
    _iinterval = static_cast<int32_t>(32 * 1000 / _target_pid_looptime_us);

    _pinterval = static_cast<int32_t>(1U << _config.sample_rate);
    if (_pinterval > _iinterval) {
        _pinterval = 0; // log only I frames if logging frequency is too low
    }

    // S-frame is written every 256*32 = 8192ms, approx every 8 seconds
    _sinterval = _iinterval * 256;

    if (_config.device == DEVICE_NONE) {
        set_state(STATE_DISABLED);
    } else if (_config.mode == MODE_ALWAYS_ON) {
        start();
    } else {
        set_state(STATE_STOPPED);
    }
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

Blackbox::state_e Blackbox::start()
{
    return start({.debug_mode = _debug_mode, .motor_count = static_cast<uint8_t>(_motor_count), .servo_count = static_cast<uint8_t>(_servo_count)});
}

Blackbox::state_e Blackbox::start(const start_t& start_parameters)
{
    return start(start_parameters, _log_select_enabled);
}

/*!
Start Blackbox logging if it is not already running. Intended to be called upon arming.
*/
Blackbox::state_e Blackbox::start(const start_t& start_parameters, uint32_t log_select_enabled)
{
    assert(start_parameters.motor_count <= blackbox_main_state_t::MAX_SUPPORTED_MOTOR_COUNT);
    assert(start_parameters.servo_count <= blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT);
    _motor_count = start_parameters.motor_count;
    _servo_count = start_parameters.servo_count;
    _debug_mode = start_parameters.debug_mode;
    _log_select_enabled = log_select_enabled;

    // We use conditional tests to decide whether or not certain fields should be logged. Since our headers
    // must always agree with the logged data, the results of these tests must not change during logging.
    // So cache those now.
    build_field_condition_cache();
    if (!_serial_device.open()) {
        set_state(STATE_DISABLED);
        return _state;
    }

#if defined(LIBRARY_BLACKBOX_USE_GPS)
    memset(&_gps_state, 0, sizeof(_gps_state));
#endif

    set_state(STATE_START);
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
        log_event(LOG_EVENT_LOG_END, nullptr);
        [[fallthrough]];
    default:
        set_state(STATE_SHUTTING_DOWN);
    }
}

void Blackbox::end_log()
{
    _serial_device.end_log(true);
}

/*
Test Motors Blackbox Logging
*/

void Blackbox::replenish_header_budget()
{
    _header_budget = static_cast<int32_t>(_serial_device.replenish_header_budget());
}

void Blackbox::start_in_test_mode()
{
    if (!_started_logging_in_test_mode) {
#if false
        if (_config.device == DEVICE_SERIAL) {
            serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
            if (sharedBlackboxAndMspPort) {
                return; // When in test mode, we cannot share the MSP and serial logger port!
            }
        }
#endif
        start();
        _started_logging_in_test_mode = true;
    }
}

void Blackbox::stop_in_test_mode()
{
    if (_started_logging_in_test_mode) {
        finish();
        _started_logging_in_test_mode = false;
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
bool Blackbox::in_motor_test_mode(const blackbox_parameter_group_t& pg)
{
    if (!_callbacks.is_armed(pg) && _callbacks.are_motors_running(pg)) {
        enum { FIVE_SECONDS_IN_MS = 5000 };
        _reset_time = time_ms() + FIVE_SECONDS_IN_MS;
        return true;
    }
    // Monitor the duration at minimum
    return (time_ms() < _reset_time);
}

void Blackbox::set_state(state_e new_state)
{
    //Perform initial setup required for the new state
    switch (new_state) {
    case STATE_START:
        break;
    case STATE_PREPARE_LOG_FILE:
        _logged_any_frames = false;
        break;
    case STATE_SEND_HEADER:
        _header_budget = 0;
        _xmit_state.header_index = 0;
#if defined(FRAMEWORK_TEST)
        _xmit_state.start_time = 0;
#else
        _xmit_state.start_time = time_ms();
#endif
        break;
    case STATE_SEND_MAIN_FIELD_HEADER:
        [[fallthrough]];
    case STATE_SEND_GPS_G_HEADER:
        [[fallthrough]];
    case STATE_SEND_GPS_H_HEADER:
        [[fallthrough]];
    case STATE_SEND_SLOW_FIELD_HEADER:
        _xmit_state.header_index = 0;
        _xmit_state.field_index = -1;
        break;
    case STATE_SEND_SYSINFO:
        _xmit_state.header_index = 0;
        break;
    case STATE_RUNNING:
        _sframe_index = _sinterval; //Force a slow frame to be written on the first iteration
#if defined(USE_FLASH_TEST_PRBS)
        // Start writing a known pattern as the running state is entered
        checkFlashStart();
#endif
        break;
    case STATE_SHUTTING_DOWN:
        _xmit_state.start_time = time_ms();
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
    _state = new_state;
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than _sinterval logging iterations
 * since the field was last logged.
 */
bool Blackbox::log_sframe_if_needed(const blackbox_parameter_group_t& pg)
{
    // Write the slow frame periodically so it can be recovered if we ever lose sync
    if (_sframe_index >= _sinterval) {
        _callbacks.load_slow_state(_slow_state, pg);
        log_sframe();
        return true;
    }

    // Only write a slow frame if it was different from the previous state
    blackbox_slow_state_t newSlowState {};
    _callbacks.load_slow_state(newSlowState, pg);
    if (memcmp(&newSlowState, &_slow_state, sizeof(_slow_state)) != 0) {
        // Use the new state as our new history
        memcpy(&_slow_state, &newSlowState, sizeof(_slow_state));
        log_sframe();
        return true;
    }
    return false;
}

void Blackbox::reset_iteration_timers()
{
    _iteration = 0;
    _loop_index = 0;
    _iframe_index = 0;
    _pframe_index = 0;
    _sframe_index = 0;
}

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
void Blackbox::advance_iteration_timers()
{
    ++_sframe_index;
    ++_iteration;

    if (++_loop_index >= _iinterval) {
        _loop_index = 0; // value of zero means IFrame will be written on next update
        ++_iframe_index;
        _pframe_index = 0;
    } else if (++_pframe_index >= _pinterval) {
        _pframe_index = 0; // value of zero means PFrame will be written on next update, if IFrame not written
    }
}

/*!
Called once every FC loop in order to log the current state
*/
void Blackbox::log_iteration(time_us_t current_time_us, const blackbox_parameter_group_t& pg)
{
    // Write a keyframe every _iinterval frames so we can resynchronise upon missing frames
    if (should_log_iframe()) { // ie _loop_index == 0
        // Don't log a slow frame if the slow data didn't change (IFrames are already large enough without adding
        // an additional item to write at the same time). Unless we're *only* logging IFrames, then we have no choice.
        if (is_only_logging_iframes()) {
            log_sframe_if_needed(pg);
        }

        _callbacks.load_main_state(*_main_state_history[0], current_time_us, pg);
        log_iframe();
    } else {
        log_event_arming_beep_if_needed(pg);
        log_event_flight_mode_if_needed(pg); // Check for FlightMode status change event

        if (should_log_pframe()) { // ie _pframe_index == 0 && _pinterval != 0
            // We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
            // So only log slow frames during loop iterations where we log a main frame.
            log_sframe_if_needed(pg);

            _callbacks.load_main_state(*_main_state_history[0], current_time_us, pg);
            log_pframe();
        }
#if defined(LIBRARY_BLACKBOX_USE_GPS)
        if (is_field_enabled(LOG_SELECT_GPS)) {
            blackbox_gps_state_t gps_stateNew {};
            _callbacks.load_gps_state(gps_stateNew, pg);

            const bool gps_stateChanged =
                gps_stateNew.satellite_count != _gps_state.satellite_count
                || gps_stateNew.latitude_degrees1E7 != _gps_state.latitude_degrees1E7
                || gps_stateNew.longitude_degrees1E7 != _gps_state.longitude_degrees1E7;

            _gps_state = gps_stateNew;

            if (should_log_hframe()) {
                _gps_home_location.latitude_degrees1E7 = _gps_state.home_latitude_degrees1E7;
                _gps_home_location.longitude_degrees1E7 = _gps_state.home_longitude_degrees1E7;
                _gps_home_location.altitude_cm = _gps_state.home_altitude_cm;
                log_hframe();
                log_gframe(current_time_us);
            } else if (gps_stateChanged) {
                //We could check for velocity changes as well but I doubt it changes independent of position
                log_gframe(current_time_us);
            }
        }
#endif
    }

    //Flush every iteration so that our runtime variance is minimized
    _serial_device.flush();
}

/*!
Called each flight loop iteration to perform blackbox logging.
*/
uint32_t Blackbox::update_log(const blackbox_parameter_group_t& pg, uint32_t current_time_us) // NOLINT(readability-function-cognitive-complexity)
{
    switch (_state) {
    case STATE_STOPPED:
        if (_callbacks.is_armed(pg)) {
            _serial_device.open();
            start();
        }
#if defined(BLACKBOX_LIBRARY_USE_FLASHFS)
        if (_callbacks.is_blackbox_erase_mode_active(pg)) {
            set_state(STATE_START_ERASE);
        }
#endif
        break;
    case STATE_START:
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE)) {
            // If we are logging battery voltage, then load_main_state to get the reference battery voltage.
            blackbox_main_state_t mainState {};
            _callbacks.load_main_state(mainState, 0, pg);
            _vbat_reference = mainState.vbat_latest;
        }

        // No need to clear the content of _main_state_history_ring since our first frame will be an intra which overwrites it

        //!!blackboxModeActivationConditionPresent = _callbacks.is_blackbox_mode_activation_condition_present();

        reset_iteration_timers();

        // Record the beeper's current idea of the last arming beep time, so that we can detect it changing when
        // it finally plays the beep for this arming event.
        _last_arming_beep = _callbacks.get_arming_beep_time_microseconds(pg);
        _last_flight_mode_flags = _callbacks.rc_mode_activation_mask(pg); // record startup status

        set_state(STATE_PREPARE_LOG_FILE);
        break;
    case STATE_PREPARE_LOG_FILE:
        if (_serial_device.begin_log()) {
            set_state(STATE_SEND_HEADER);
        }
        break;
    case STATE_SEND_HEADER:
        _header_budget = static_cast<int32_t>(_serial_device.replenish_header_budget());
        //On entry of this state, _xmit_state.header_index is 0 and start_time is initialized
        // Give the UART time to initialize
        //if (time_ms() < _xmit_state.start_time + 100) {
        //    break;
        //}
        if (write_header() == WRITE_COMPLETE) { // keep on writing chunks of the header until it returns false, signalling completion
            set_state(STATE_SEND_MAIN_FIELD_HEADER);
        }
        break;
    case STATE_SEND_MAIN_FIELD_HEADER:
        _header_budget = static_cast<int32_t>(_serial_device.replenish_header_budget());
        // On entry of this state, _xmit_state.header_index is 0 and _xmit_state.field_index is -1
        if (write_field_header_main() == WRITE_COMPLETE) { // keep on writing chunks of the main field header until it returns false, signalling completion
#if defined(LIBRARY_BLACKBOX_USE_GPS)
            set_state(is_field_enabled(LOG_SELECT_GPS) ? STATE_SEND_GPS_H_HEADER : STATE_SEND_SLOW_FIELD_HEADER);
#else
            set_state(STATE_SEND_SLOW_FIELD_HEADER);
#endif
        }
        break;
#if defined(LIBRARY_BLACKBOX_USE_GPS)
    case STATE_SEND_GPS_H_HEADER:
        _header_budget = static_cast<int32_t>(_serial_device.replenish_header_budget());
        if (write_field_header_gps_h() == WRITE_COMPLETE) {
            set_state(STATE_SEND_GPS_G_HEADER);
        }
        break;
    case STATE_SEND_GPS_G_HEADER:
        _header_budget = static_cast<int32_t>(_serial_device.replenish_header_budget());
        if (write_field_header_gps_g() == WRITE_COMPLETE) {
            set_state(STATE_SEND_SLOW_FIELD_HEADER);
        }
        break;
#endif
    case STATE_SEND_SLOW_FIELD_HEADER:
        _header_budget = static_cast<int32_t>(_serial_device.replenish_header_budget());
        // On entry of this state, _xmit_state.header_index is 0 and _xmit_state.field_index is -1
        if (write_field_header_slow() == WRITE_COMPLETE) { // keep on writing chunks of the slow field header until it returns false, signalling completion
            _cache_flush_next_state = STATE_SEND_SYSINFO;
            set_state(STATE_CACHE_FLUSH);
        }
        break;
    case STATE_SEND_SYSINFO:
        _header_budget = static_cast<int32_t>(_serial_device.replenish_header_budget());
        //On entry of this state, _xmit_state.header_index is 0

        //Keep writing chunks of the system info headers until it returns true to signal completion
        if (write_system_information(pg) == WRITE_COMPLETE) {
            /*
             * Wait for header buffers to drain completely before data logging begins to ensure reliable header delivery
             * (overflowing circular buffers causes all data to be discarded, so the first few logged iterations
             * could wipe out the end of the header if we weren't careful)
             */
            _cache_flush_next_state = STATE_RUNNING;
            set_state(STATE_CACHE_FLUSH);
        }
        break;
    case STATE_CACHE_FLUSH:
        // Flush the cache and wait until all possible entries have been written to the media
        if (_serial_device.flush_force_complete()) {
            set_state(_cache_flush_next_state);
        }
        break;
    case STATE_PAUSED:
        // Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
        if (_callbacks.is_blackbox_mode_active(pg) && should_log_iframe()) {
            // Write a log entry so the decoder is aware that our large time/iteration skip is intended
            //flightLogEvent_logging_resume_t resume {
            //    .log_iteration = _iteration,
            //    .current_time = current_time_us
            //};
            const log_event_data_u resume = {
                .logging_resume = {
                    .log_iteration = _iteration,
                    .current_time = current_time_us
                }
            };
            log_event(LOG_EVENT_LOGGING_RESUME, &resume);
            set_state(STATE_RUNNING);

            log_iteration(current_time_us, pg);
        }
        // Keep the logging timers ticking so our log iteration continues to advance
        advance_iteration_timers();
        break;
    case STATE_RUNNING:
        // On entry to this state, _iteration, _pframe_index and _iframe_index are reset to 0
        // Prevent the Pausing of the log on the mode switch if in Motor Test Mode
        if (_callbacks.is_blackbox_mode_activation_condition_present(pg) && !_callbacks.is_blackbox_mode_active(pg) && !_started_logging_in_test_mode) {
            set_state(STATE_PAUSED);
        } else {
            log_iteration(current_time_us, pg);
        }
        advance_iteration_timers();
        break;
    case STATE_SHUTTING_DOWN:
        //On entry of this state, start_time is set
        /*
         * Wait for the log we've transmitted to make its way to the logger before we release the serial port,
         * since releasing the port clears the Tx buffer.
         *
         * Don't wait longer than it could possibly take if something funky happens.
         */
        if (_serial_device.end_log(_logged_any_frames) && (time_ms() > _xmit_state.start_time + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS || _serial_device.flush_force())) { // cppcheck-suppress unsignedLessThanZero
            _serial_device.close();
            set_state(STATE_STOPPED);
        }
        break;
#if defined(BLACKBOX_LIBRARY_USE_FLASHFS)
    case STATE_START_ERASE:
        _serial_device.erase_all();
        set_state(STATE_ERASING);
        _callbacks.beep(pg);
        break;
    case STATE_ERASING:
        if (_serial_device.is_erased()) {
            //Done erasing
            set_state(STATE_ERASED);
            _callbacks.beep(pg);
        }
        break;
    case STATE_ERASED:
        if (!_callbacks.is_blackbox_erase_mode_active(pg)) {
            set_state(STATE_STOPPED);
        }
        break;
#endif
    default:
        break;
    }

    // Did we run out of room on the device? Stop!
    if (_serial_device.is_device_full()) {
#if defined(BLACKBOX_LIBRARY_USE_FLASHFS)
        if (_state != STATE_ERASING && _state != STATE_START_ERASE && _state != STATE_ERASED)
#endif
        {
            set_state(STATE_STOPPED);
            // ensure we reset the test mode flag if we stop due to full memory card
            _started_logging_in_test_mode = false;
        }
    } else { // Only log in test mode if there is room!
        switch (_config.mode) {
        case MODE_MOTOR_TEST:
            // Handle Motor Test Mode
            if (in_motor_test_mode(pg)) {
                if (_state == STATE_STOPPED) {
                    start_in_test_mode();
                }
            } else {
                if (_state != STATE_STOPPED) {
                    stop_in_test_mode();
                }
            }
            break;
        case MODE_ALWAYS_ON:
            if (_state == STATE_STOPPED) {
                start_in_test_mode();
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

uint8_t Blackbox::calculate_sample_rate(uint16_t p_ration) const
{
    return static_cast<uint8_t>(llog2(32000 / (_target_pid_looptime_us * p_ration)));  // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
}

/*!
Return true if it is safe to edit the Blackbox configuration.
*/
bool Blackbox::may_edit_config()
{
    return _state <= STATE_STOPPED;
}

void Blackbox::log_iframe() // NOLINT(readability-function-cognitive-complexity)
{
    _encoder.beginFrame('I');

    _encoder.writeUnsignedVB(_iteration);

    const blackbox_main_state_t* mainState = _main_state_history[0];

    _encoder.writeUnsignedVB(mainState->time_us);

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID)) {
        _encoder.writeSignedVBArray(&mainState->axis_pid_p[0], RPY_AXIS_COUNT);
        _encoder.writeSignedVBArray(&mainState->axis_pid_i[0], RPY_AXIS_COUNT);

        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_D_ROLL)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[0]);
        }
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_D_PITCH)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[1]);
        }
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_D_YAW)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[2]);
        }

        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_K)) {
            _encoder.writeSignedVBArray(&mainState->axis_pid_k[0], RPY_AXIS_COUNT);
        }

        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_S_ROLL)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[0]);
        }
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_S_PITCH)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[1]);
        }
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_S_YAW)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[2]);
        }
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS)) {
        // Write roll, pitch and yaw first, these are signed values in the range [-500,500]
        _encoder.writeSigned16VBArray(&mainState->rc_command[0], 3);

        // Write the throttle separately from the rest of the RC data as it's unsigned.
        // Throttle lies in range [PWM_RANGE_MIN,PWM_RANGE_MAX], ie [1000,2000]
        enum { ROLL = 0, PITCH, YAW, THROTTLE };
        _encoder.writeUnsignedVB(static_cast<uint32_t>(mainState->rc_command[THROTTLE]));
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT)) {
        // Write setpoint roll, pitch, yaw, and throttle
        _encoder.writeSigned16VBArray(&mainState->setpoint[0], 4);
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE)) {
        //Our voltage is expected to decrease over the course of the flight, so store our difference from
        //the reference:
        // Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
        enum { LEAST_SIGNIFICANT_14_BITS = 0x3FFF };
        _encoder.writeUnsignedVB((_vbat_reference - mainState->vbat_latest) & LEAST_SIGNIFICANT_14_BITS);
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_CURRENT)) {
        // 12bit value directly from ADC
        _encoder.writeSignedVB(mainState->amperage_latest);
    }

#if defined(LIBRARY_BLACKBOX_USE_MAGNETOMETER)
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER)) {
        _encoder.writeSigned16VBArray(&mainState->mag_adc[0], XYZ_AXIS_COUNT);
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_BAROMETER)
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_BAROMETER)) {
        _encoder.writeSignedVB(mainState->baro_altitude);
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_RANGEFINDER)
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER)) {
        _encoder.writeSignedVB(mainState->surface_raw);
    }
#endif

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        _encoder.writeUnsignedVB(mainState->rssi);
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_GYRO)) {
        _encoder.writeSigned16VBArray(&mainState->gyro_adc[0], XYZ_AXIS_COUNT);
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED)) {
        _encoder.writeSigned16VBArray(&mainState->gyro_unfiltered[0], XYZ_AXIS_COUNT);
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_ACC)) {
        _encoder.writeSigned16VBArray(&mainState->acc_adc[0], XYZ_AXIS_COUNT);
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_ATTITUDE)) {
        _encoder.writeSigned16VBArray(&mainState->orientation[0], XYZ_AXIS_COUNT);
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_DEBUG)) {
        _encoder.writeSigned16VBArray(&mainState->debug[0], blackbox_main_state_t::DEBUG_VALUE_COUNT);
    }

    if (is_field_enabled(LOG_SELECT_MOTOR)) {
        //Motors can be below minimum output when disarmed, but that doesn't happen much
        _encoder.writeUnsignedVB(static_cast<uint32_t>(mainState->motor[0] - static_cast<int>(_motor_output_low)));

        //Motors tend to be similar to each other so use the first motor's value as a predictor of the others
        for (size_t ii = 1; ii < _motor_count; ++ii) {
            _encoder.writeSignedVB(mainState->motor[ii] - mainState->motor[0]);
        }
    }
#if defined(LIBRARY_BLACKBOX_USE_SERVOS)
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_SERVOS)) {
        std::array <int32_t, blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT> out;
        for (size_t ii = 0; ii < blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
            out[ii] = mainState->servo[ii] - 1500; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        }
        _encoder.writeTag8_8SVB(&out[0], blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT);
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_DSHOT_TELEMETRY)
    if (is_field_enabled(LOG_SELECT_MOTOR_RPM)) {
        for (size_t ii = 0; ii < _motor_count; ++ii) {
            _encoder.writeUnsignedVB(static_cast<uint32_t>(mainState->erpm[ii]));
        }
    }
#endif

    _encoder.endframe();
// 2=1
// 1=0
// 0=2
    blackbox_main_state_t* const history2Save = _main_state_history[2];

    // The current state becomes the new "before" state
    _main_state_history[1] = _main_state_history[0];
    // And since we have no other history, we also use it for the "before, before" state
    _main_state_history[2] = _main_state_history[0];
    // And advance the current state over to a blank space ready to be filled
    // _main_state_history[0] = ((_main_state_history[0] - &_main_state_history_ring[1]) % 3) + &_main_state_history_ring[0];
    _main_state_history[0] = history2Save;

    _logged_any_frames = true;
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

void Blackbox::log_pframe() // NOLINT(readability-function-cognitive-complexity)
{
    _encoder.beginFrame('P');
    const blackbox_main_state_t* mainState = _main_state_history[0];
    const blackbox_main_state_t* previousMainState = _main_state_history[1];

    //No need to store iteration count since its delta is always 1

    // Since the difference between the difference between successive times will be nearly zero (due to consistent
    // looptime spacing), use second-order differences.
    _encoder.writeSignedVB(static_cast<int32_t>(mainState->time_us - 2 * _main_state_history[1]->time_us + _main_state_history[2]->time_us));

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID)) {
        //arraySubInt32(&deltas[0], &mainState->axis_pid_p[0], &previousMainState->axis_pid_p[0], RPY_AXIS_COUNT);
        std::array<int32_t, RPY_AXIS_COUNT> deltas = mainState->axis_pid_p - previousMainState->axis_pid_p;
        _encoder.writeSignedVBArray(&deltas[0], RPY_AXIS_COUNT);

        // The PID I field changes very slowly, most of the time +-2, so use an encoding
        // that can pack all three fields into one byte in that situation.
        deltas = mainState->axis_pid_i - previousMainState->axis_pid_i;
        _encoder.writeTag2_3S32(&deltas[0]);

        // The PID D term is frequently set to zero for yaw, which makes the result from the calculation
        // always zero. So don't bother recording D results when PID D terms are zero.
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_D_ROLL)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[0] - previousMainState->axis_pid_d[0]);
        }
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_D_PITCH)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[1] - previousMainState->axis_pid_d[1]);
        }
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_D_YAW)) {
            _encoder.writeSignedVB(mainState->axis_pid_d[2] - previousMainState->axis_pid_d[2]);
        }

        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_K)) {
            deltas = mainState->axis_pid_k - previousMainState->axis_pid_k;
            _encoder.writeSignedVBArray(&deltas[0], RPY_AXIS_COUNT);
        }

        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_S_ROLL)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[0] - previousMainState->axis_pid_s[0]);
        }
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_S_PITCH)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[1] - previousMainState->axis_pid_s[1]);
        }
        if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_PID_S_YAW)) {
            _encoder.writeSignedVB(mainState->axis_pid_s[2] - previousMainState->axis_pid_s[2]);
        }
    }

    // RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS)) {
        const std::array<int32_t, 4> deltas = mainState->rc_command - previousMainState->rc_command;
        _encoder.writeTag8_4S16(&deltas[0]);
    }
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT)) {
        const std::array<int32_t, 4> deltas = mainState->setpoint - previousMainState->setpoint;
        _encoder.writeTag8_4S16(&deltas[0]);
    }

    enum { MAX_DELTA_COUNT = 8 };
    std::array<int32_t, MAX_DELTA_COUNT> deltas;
    //Check for sensors that are updated periodically (so deltas are normally zero)
    size_t optionalFieldCount = 0;

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE)) {
        deltas[optionalFieldCount++] = mainState->vbat_latest - previousMainState->vbat_latest;
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_CURRENT)) {
        deltas[optionalFieldCount++] = mainState->amperage_latest - previousMainState->amperage_latest;
    }

#if defined(LIBRARY_BLACKBOX_USE_MAGNETOMETER)
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            deltas[optionalFieldCount++] = mainState->mag_adc[ii] - previousMainState->mag_adc[ii];
        }
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_BAROMETER)
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_BAROMETER)) {
        deltas[optionalFieldCount++] = mainState->baro_altitude - previousMainState->baro_altitude;
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_RANGEFINDER)
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER)) {
        deltas[optionalFieldCount++] = mainState->surface_raw - previousMainState->surface_raw;
    }
#endif

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_RSSI)) {
        deltas[optionalFieldCount++] = mainState->rssi - previousMainState->rssi;
    }

    _encoder.writeTag8_8SVB(&deltas[0], optionalFieldCount);

    //Since gyros, accelerometers and motors are noisy, base their predictions on the average of the history:
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_GYRO)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_main_state_history[1]->gyro_adc[ii] + _main_state_history[2]->gyro_adc[ii]) / 2;
            _encoder.writeSignedVB(mainState->gyro_adc[ii] - predictor);
        }
    }
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_main_state_history[1]->gyro_unfiltered[ii] + _main_state_history[2]->gyro_unfiltered[ii]) / 2;
            _encoder.writeSignedVB(mainState->gyro_unfiltered[ii] - predictor);
        }
    }
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_ACC)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_main_state_history[1]->acc_adc[ii] + _main_state_history[2]->acc_adc[ii]) / 2;
            _encoder.writeSignedVB(mainState->acc_adc[ii] - predictor);
        }
    }
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_ATTITUDE)) {
        for (size_t ii = 0; ii < XYZ_AXIS_COUNT; ++ii) {
            const int32_t predictor = (_main_state_history[1]->orientation[ii] + _main_state_history[2]->orientation[ii]) / 2;
            _encoder.writeSignedVB(mainState->orientation[ii] - predictor);
        }
    }

    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_DEBUG)) {
        for (size_t ii = 0; ii < blackbox_main_state_t::DEBUG_VALUE_COUNT; ++ii) {
            const int32_t predictor = (_main_state_history[1]->debug[ii] + _main_state_history[2]->debug[ii]) / 2;
            _encoder.writeSignedVB(mainState->debug[ii] - predictor);
        }
    }

    if (is_field_enabled(LOG_SELECT_MOTOR)) {
        for (size_t ii = 0; ii < _motor_count; ++ii) {
            const int32_t predictor = (_main_state_history[1]->motor[ii] + _main_state_history[2]->motor[ii]) / 2;
            _encoder.writeSignedVB(mainState->motor[ii] - predictor);
        }
    }

#if defined(LIBRARY_BLACKBOX_USE_SERVOS)
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_SERVOS)) {
        std::array <int32_t, blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT> out;
        for (size_t ii = 0; ii < blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
            out[ii] = mainState->servo[ii] - 1500; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        }
        _encoder.writeTag8_8SVB(&out[0], blackbox_main_state_t::MAX_SUPPORTED_SERVO_COUNT);
    }
#endif

#if defined(LIBRARY_BLACKBOX_USE_DSHOT_TELEMETRY)
    if (is_field_enabled(LOG_SELECT_MOTOR_RPM)) {
        for (size_t ii = 0; ii < _motor_count; ++ii) {
            _encoder.writeSignedVB(mainState->erpm[ii] - previousMainState->erpm[ii]);
        }
    }
#endif
    //Rotate our history buffers
// 2=1
// 1=0
// 0=2
    blackbox_main_state_t* const history2Save = _main_state_history[2];
    _main_state_history[2] = _main_state_history[1];
    _main_state_history[1] = _main_state_history[0];
    //_main_state_history[0] = &_main_state_history_ring[0] + ((_main_state_history[0] - &_main_state_history_ring[1]) % 3);
    _main_state_history[0] = history2Save;

    _logged_any_frames = true;
    _encoder.endframe();
}

/*!
Write the contents of the global "_slow_state" to the log as an S frame.
Because this data is logged so infrequently, delta updates are not reasonable, so we log independent frames.
*/
void Blackbox::log_sframe()
{
    _encoder.beginFrame('S');

    _encoder.writeUnsignedVB(_slow_state.flight_mode_flags);
    _encoder.writeUnsignedVB(_slow_state.state_flags);

    // Most of the time these three values will be able to pack into one byte.
    const std::array<int32_t, 3> values {
        _slow_state.failsafe_phase,
        _slow_state.rx_signal_received ? 1 : 0,
        _slow_state.rx_flight_channel_is_valid ? 1 : 0
    };

    _encoder.writeTag2_3S32(&values[0]);

    _sframe_index = 0;

    _encoder.endframe();
}

#if defined(LIBRARY_BLACKBOX_USE_GPS)
/*!
If the GPS home point has been updated, or every 128 I-frames (~10 seconds), write the
GPS home position.

We write it periodically so that if one Home Frame goes missing, the GPS coordinates can
still be interpreted correctly.
*/
bool Blackbox::should_log_hframe() const
{
    if ((_gps_home_location.latitude_degrees1E7 != _gps_state.home_latitude_degrees1E7
         || _gps_home_location.longitude_degrees1E7 != _gps_state.home_longitude_degrees1E7
         || (_pframe_index == _iinterval / 2 && _iframe_index % 128 == 0)) // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)
        && is_field_enabled(LOG_SELECT_GPS)) {
        return true; // NOLINT(readability-simplify-boolean-expr)
    }
    return false;
}

void Blackbox::log_hframe()
{
    _encoder.beginFrame('H');

    _encoder.writeSignedVB(_gps_state.home_latitude_degrees1E7);
    _encoder.writeSignedVB(_gps_state.home_longitude_degrees1E7);
    _encoder.writeSignedVB(_gps_state.home_altitude_cm / 10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)

    _encoder.endframe();
}

void Blackbox::log_gframe(time_us_t current_time_us)
{
    _encoder.beginFrame('G');

    // If we're logging every frame, then a GPS frame always appears just after a frame with the
    // current_time timestamp in the log, so the reader can just use that timestamp for the GPS frame.
    // If we're not logging every frame, we need to store the time of this GPS frame.
    if (test_field_condition(FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME)) {
        // Predict the time of the last frame in the main log
        _encoder.writeUnsignedVB(current_time_us - _main_state_history[1]->time_us);
    }

    _encoder.writeUnsignedVB(_gps_state.satellite_count);
    _encoder.writeSignedVB(_gps_state.latitude_degrees1E7 - _gps_home_location.latitude_degrees1E7);
    _encoder.writeSignedVB(_gps_state.longitude_degrees1E7 - _gps_home_location.longitude_degrees1E7);
    // log altitude in increments of 0.1m
    _encoder.writeSignedVB(_gps_state.altitude_cm / 10); // NOLINT(cppcoreguidelines-avoid-magic-numbers,modernize-deprecated-headers,readability-magic-numbers)

    if (_config.gps_use_3d_speed) {
        _encoder.writeUnsignedVB(static_cast<uint32_t>(_gps_state.speed3d_cmps));
    } else {
        _encoder.writeUnsignedVB(static_cast<uint32_t>(_gps_state.ground_speed_cmps));
    }

    _encoder.writeUnsignedVB(static_cast<uint32_t>(_gps_state.ground_course_deci_degrees));

    _encoder.writeSignedVB(_gps_state.velocity_north_cmps);
    _encoder.writeSignedVB(_gps_state.velocity_east_cmps);
    _encoder.writeSignedVB(_gps_state.velocity_down_cmps);

    _encoder.endframe();
}
#endif // LIBRARY_BLACKBOX_USE_GPS


/*!
Write the given event to the log immediately
*/
bool Blackbox::log_event(log_event_e event, const log_event_data_u* data)
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
        _encoder.writeUnsignedVB(data->sync_beep.time);
        break;
    case LOG_EVENT_FLIGHTMODE: // New flightmode flags write
        _encoder.writeUnsignedVB(data->flight_mode.flags);
        _encoder.writeUnsignedVB(data->flight_mode.last_flags);
        break;
    case LOG_EVENT_DISARM:
        _encoder.writeUnsignedVB(data->disarm.reason);
        break;
    case LOG_EVENT_INFLIGHT_ADJUSTMENT:
        if (data->inflight_adjustment.float_flag) {
            enum { LOG_EVENT_INFLIGHT_ADJUSTMENT_FLOAT_VALUE_FLAG = 128 };
            _encoder.write(data->inflight_adjustment.adjustment + LOG_EVENT_INFLIGHT_ADJUSTMENT_FLOAT_VALUE_FLAG);
            _encoder.writeFloat(data->inflight_adjustment.new_float_value);
        } else {
            _encoder.write(data->inflight_adjustment.adjustment);
            _encoder.writeSignedVB(data->inflight_adjustment.new_value);
        }
        break;
    case LOG_EVENT_LOGGING_RESUME:
        _encoder.writeUnsignedVB(data->logging_resume.log_iteration);
        _encoder.writeUnsignedVB(data->logging_resume.current_time);
        break;
    case LOG_EVENT_LOG_END:
        // data is nullptr for LOG_EVENT_LOG_END
        header_write_string("End of log");
        _encoder.write(0);
        break;
    default:
        break;
    }

    _encoder.endframe();

    return true;
}

/* If an arming beep has played since it was last logged, write the time of the arming beep to the log as a synchronization point */
void Blackbox::log_event_arming_beep_if_needed(const blackbox_parameter_group_t& pg)
{
    // Use != so that we can still detect a change if the counter wraps
    const uint32_t armingBeepTimeMicroseconds = _callbacks.get_arming_beep_time_microseconds(pg);
    if (armingBeepTimeMicroseconds != _last_arming_beep) {
        _last_arming_beep = armingBeepTimeMicroseconds;
        const log_event_data_u eventData {
            .sync_beep = {
                .time  = _last_arming_beep
            }
        };
        log_event(LOG_EVENT_SYNC_BEEP, &eventData);
    }
}

/* monitor the flight mode event status and trigger an event record if the state changes */
void Blackbox::log_event_flight_mode_if_needed(const blackbox_parameter_group_t& pg)
{
    // Use != so that we can still detect a change if the counter wraps
#if false
    if (memcmp(&_callbacks._rc_mode_activation_mask, &_last_flight_mode_flags, sizeof(_last_flight_mode_flags))) {
        static flightLogEvent_flight_mode_t eventData {}; // Add new data for current flight mode flags
        eventData.last_flags = _last_flight_mode_flags;
        memcpy(&_last_flight_mode_flags, &_callbacks._rc_mode_activation_mask, sizeof(_last_flight_mode_flags));
        memcpy(&eventData.flags, &_callbacks._rc_mode_activation_mask, sizeof(eventData.flags));
        log_event(LOG_EVENT_FLIGHTMODE, reinterpret_cast<flightLogEventData_u*>(&eventData)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    }
#endif
    const uint32_t rc_mode_activation_mask = _callbacks.rc_mode_activation_mask(pg);
    if (rc_mode_activation_mask != _last_flight_mode_flags) {
        const log_event_data_u eventData = {
            .flight_mode = {
                .flags = rc_mode_activation_mask,
                .last_flags = _last_flight_mode_flags
            }
        };
        _last_flight_mode_flags = rc_mode_activation_mask;
        log_event(LOG_EVENT_FLIGHTMODE, &eventData);
    }
}
