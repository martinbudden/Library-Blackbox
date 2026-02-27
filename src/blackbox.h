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

#include "blackbox_callbacks_base.h"
#include "blackbox_encoder.h"

#include <bitset>

class BlackboxCallbacksBase;
class BlackboxSerialDevice;
struct blackbox_simple_field_definition_t;
struct blackbox_parameter_group_t;


class Blackbox {
public:
    virtual ~Blackbox() = default;
    Blackbox(uint32_t pidLoopTimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serial_device) :
        _serial_device(serial_device),
        _encoder(_serial_device),
        _callbacks(callbacks),
        _target_pid_looptime_us(pidLoopTimeUs)
        {}
public:
    // Ideally, each iteration in which we are logging headers would write a similar amount of data to the device as a
    // regular logging iteration. This way we won't hog the CPU by making a gigantic write:

    enum { XYZ_AXIS_COUNT = 3 };
    enum { RPY_AXIS_COUNT = 3 };
    enum { BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION = 64 };

    enum device_e : uint8_t {
        DEVICE_NONE = 0,
        DEVICE_FLASH = 1,
        DEVICE_SDCARD = 2,
        DEVICE_SERIAL = 3
    };

    enum mode_e : uint8_t {
        MODE_NORMAL = 0,
        MODE_MOTOR_TEST,
        MODE_ALWAYS_ON
    };

    enum sample_rate_e : uint8_t { // Sample rate is 1/(2^BlackboxSampleRate)
        RATE_ONE = 0,
        RATE_HALF,
        RATE_QUARTER,
        RATE_8TH,
        RATE_16TH
    };

    enum log_field_select_e { // no more than 32
        LOG_SELECT_DEBUG            = 0x0001,
        LOG_SELECT_PID              = 0x0002,
        LOG_SELECT_PID_DTERM_ROLL   = 0x0004,
        LOG_SELECT_PID_DTERM_PITCH  = 0x0008,
        LOG_SELECT_PID_DTERM_YAW    = 0x0010,
        LOG_SELECT_PID_STERM_ROLL   = 0x0020,
        LOG_SELECT_PID_STERM_PITCH  = 0x0040,
        LOG_SELECT_PID_STERM_YAW    = 0x0080,
        LOG_SELECT_PID_KTERM        = 0x0100,
        LOG_SELECT_SETPOINT         = 0x0200,
        LOG_SELECT_RC_COMMANDS      = 0x0400,
        LOG_SELECT_RSSI             = 0x0800,
        LOG_SELECT_GYRO             = 0x1000,
        LOG_SELECT_GYRO_UNFILTERED  = 0x2000,
        LOG_SELECT_ACCELEROMETER    = 0x4000,
        LOG_SELECT_ATTITUDE         = 0x8000,
        LOG_SELECT_MAGNETOMETER    = 0x10000,
        LOG_SELECT_MOTOR           = 0x20000,
        LOG_SELECT_MOTOR_RPM       = 0x40000,
        LOG_SELECT_SERVO           = 0x80000,
        LOG_SELECT_BATTERY_VOLTAGE= 0x100000,
        LOG_SELECT_BATTERY_CURRENT= 0x200000,
        LOG_SELECT_BAROMETER      = 0x400000,
        LOG_SELECT_RANGEFINDER    = 0x800000,
        LOG_SELECT_GPS           = 0x1000000,
    };

    enum state_e {
        STATE_DISABLED = 0,
        STATE_STOPPED,
        STATE_START,
        STATE_PREPARE_LOG_FILE,
        STATE_SEND_HEADER,
        STATE_SEND_MAIN_FIELD_HEADER,
        STATE_SEND_GPS_H_HEADER,
        STATE_SEND_GPS_G_HEADER,
        STATE_SEND_SLOW_FIELD_HEADER,
        STATE_SEND_SYSINFO,
        STATE_CACHE_FLUSH,
        STATE_PAUSED,
        STATE_RUNNING,
        STATE_SHUTTING_DOWN,
        STATE_START_ERASE,
        STATE_ERASING,
        STATE_ERASED
    };
public:
    typedef uint32_t time_us_t;
    typedef uint32_t time_ms_t;

    struct config_t {
        sample_rate_e sample_rate;
        device_e device;
        mode_e mode;
        bool gps_use_3d_speed;
        uint32_t fields_disabled_mask;
    };
    struct start_t {
        uint16_t debug_mode;
        uint8_t motor_count;
        uint8_t servo_count;
    };
    struct xmit_state_t {
        uint32_t headerIndex;
        int32_t fieldIndex;
        uint32_t startTime;
    };
    struct log_event_sync_beep_t {
        uint32_t time;
    };
    struct log_event_disarm_t {
        uint32_t reason;
    };
    struct log_event_flightMode_t { // New Event Data type
        uint32_t flags;
        uint32_t lastFlags;
    };
    struct log_event_inflight_adjustment_t {
        int32_t newValue;
        float newFloatValue;
        uint8_t adjustment;
        bool floatFlag;
    };
    struct log_event_logging_resume_t {
        uint32_t log_iteration;
        uint32_t currentTime;
    };
    union log_event_data_u {
        log_event_sync_beep_t syncBeep;
        log_event_flightMode_t flightMode; // New event data
        log_event_disarm_t disarm;
        log_event_inflight_adjustment_t inflightAdjustment;
        log_event_logging_resume_t loggingResume;
    };
    enum log_event_e {
        LOG_EVENT_SYNC_BEEP = 0,
        LOG_EVENT_AUTOTUNE_CYCLE_START = 10,   // UNUSED
        LOG_EVENT_AUTOTUNE_CYCLE_RESULT = 11,  // UNUSED
        LOG_EVENT_AUTOTUNE_TARGETS = 12,       // UNUSED
        LOG_EVENT_INFLIGHT_ADJUSTMENT = 13,
        LOG_EVENT_LOGGING_RESUME = 14,
        LOG_EVENT_DISARM = 15,
        LOG_EVENT_FLIGHTMODE = 30, // Add new event type for flight mode status.
        LOG_EVENT_LOG_END = 255
    };
    struct log_event_t {
        log_event_e event;
        log_event_data_u data;
    };
    struct gps_location_t {
        int32_t longitude_degrees1E7;   // longitude in degrees * 1e+7
        int32_t latitude_degrees1E7;    // latitude in degrees * 1e+7
        int32_t altitude_cm;            // altitude in cm
    };
    // Only available on U-blox protocol
    struct gps_accuracy_t {
        uint32_t horizontalAccuracy_mm;
        uint32_t verticalAccuracy_mm;
        uint32_t speedAccuracy_mmps;
    };
public:
    enum write_e { WRITE_COMPLETE, WRITE_NOT_COMPLETE };
    virtual write_e write_system_information(const blackbox_parameter_group_t& pg) = 0;

    uint32_t update_log(const blackbox_parameter_group_t& pg, uint32_t current_time_us); // main loop function, updates the blackbox log

    bool header_reserve_buffer_space();
    size_t printfv(const char* fmt, va_list va);
    size_t printf(const char* fmt, ...);
    size_t header_printf_header_line(const char* name, const char* fmt, ...);
    size_t header_printf(const char* fmt, ...);
    void header_write(uint8_t value);
    size_t header_write_string(const char* s);

    write_e write_header();
    write_e write_field_header_main();
    write_e write_field_header_simple(char field_char, const blackbox_simple_field_definition_t* fields, int32_t field_count);
    write_e write_field_header_slow();
    write_e write_field_header_gps_h();
    write_e write_field_header_gps_g();

    static inline bool is_field_enabled(uint32_t enabled_mask, log_field_select_e field) { return (enabled_mask & static_cast<uint32_t>(field)) != 0; }
    inline bool is_field_enabled(log_field_select_e field) const { return is_field_enabled(_log_select_enabled, field); }

    void build_field_condition_cache();
    bool test_field_condition_uncached(uint8_t condition) const;
    inline bool test_field_condition(uint8_t condition) const { return _condition_cache.test(condition); }

    bool is_only_logging_i_frames() const { return _p_interval == 0; }
    bool should_log_i_frame() const { return _loop_index == 0; }
    bool should_log_p_frame() const { return _p_frame_index == 0 && _p_interval != 0; }
    bool should_log_h_frame() const;

    void log_i_frame(); // Intraframe, keyframe
    void log_p_frame(); // Interframe, delta frame
    void log_s_frame(); // Slow frame
    bool log_s_frame_if_needed(const blackbox_parameter_group_t& pg);
    void log_h_frame(); // GPS home frame
    void log_g_frame(time_us_t current_time_us); // GPS frame
    bool log_event(log_event_e event, const log_event_data_u* data); // E-frame
    void log_event_arming_beep_if_needed(const blackbox_parameter_group_t& pg); // E-frame
    void log_event_flight_mode_if_needed(const blackbox_parameter_group_t& pg); // E-frame

    void log_iteration(time_us_t current_time_us, const blackbox_parameter_group_t& pg);
    void advance_iteration_timers();
    void reset_iteration_timers();
    void set_state(state_e newState);

    void init(const config_t& config);
    state_e start(const start_t& start_parameters, uint32_t log_select_enabled);
    state_e start(const start_t& start_parameters);
    state_e start();
    void finish();
    void end_log();
    void start_in_test_mode();
    void stop_in_test_mode();

    const config_t& get_config() const { return _config; }
    uint16_t get_debug_mode() const { return _debug_mode; }
    int32_t get_i_interval() const { return _i_interval; }
    int32_t get_p_interval() const { return _p_interval; }
    int32_t get_s_interval() const { return _s_interval; }
    bool may_edit_config(void);

    uint8_t calculate_sample_rate(uint16_t p_ration) const;
    int calculate_p_denominator(int rate_numerator, int rate_denominator) { return _i_interval * rate_numerator / rate_denominator; }

    bool in_motor_test_mode(const blackbox_parameter_group_t& pg);

    void replenish_header_budget();
protected:
    BlackboxSerialDevice& _serial_device;
    BlackboxEncoder _encoder;
    BlackboxCallbacksBase& _callbacks;
    size_t _motor_count;
    size_t _servo_count;
    uint32_t _log_select_enabled {};
    float _motor_output_low { 0.0F }; //!!TODO allow this to be set
    uint32_t _reset_time = 0;
    uint16_t _debug_mode;

    config_t _config {
        .sample_rate = RATE_ONE,
        .device = DEVICE_SDCARD,
        .mode = MODE_NORMAL,
        .gps_use_3d_speed = false,
        .fields_disabled_mask = 0
    };
    int32_t _header_budget {};
    // _target_pid_looptime_us is 1000 for 1kHz loop, 500 for 2kHz loop etc, _target_pid_looptime_us is rounded for short looptimes
    uint32_t _target_pid_looptime_us; // time in microseconds
    state_e _state = STATE_DISABLED;

    bool _started_logging_in_test_mode = false;
    uint32_t _last_arming_beep = 0;
    uint32_t _last_flight_mode_flags = 0; // New event tracking of flight modes
// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
    std::bitset<64> _condition_cache {};

    uint32_t _iteration {};
    int32_t _loop_index {};
    int32_t _p_frame_index {};
    int32_t _i_frame_index {}; // use to determine if HFrames should be logged
    int32_t _s_frame_index {};
    int32_t _i_interval = 0; //!< number of flight loop iterations before logging I-frame, typically 32 for 1kHz loop, 64 for 2kHz loop etc
    int32_t _p_interval = 0; //!< number of flight loop iterations before logging P-frame
    int32_t _s_interval = 0;
    bool _logged_any_frames {};
    // We store voltages in I-frames relative to _vbat_reference, which was the voltage when the blackbox was activated.
    // This helps out since the voltage is only expected to fall from that point and we can reduce our diffs to encode.
    uint16_t _vbat_reference {};
    xmit_state_t  _xmit_state {};
    state_e _cache_flush_next_state {};
#if defined(LIBRARY_BLACKBOX_USE_GPS)
    gps_location_t _gps_home_location {};
    blackbox_gps_state_t _gps_state {};
#endif
    blackbox_slow_state_t _slow_state {};
    // Keep a history of length 2, plus a buffer to store the new values into
    std::array<blackbox_main_state_t, 3> _main_state_history_ring {};
    // These point into _main_state_history_ring, use them to know where to store history of a given age (0, 1 or 2 generations old)
    std::array<blackbox_main_state_t*, 3> _main_state_history {
        &_main_state_history_ring[0],
        &_main_state_history_ring[1],
        &_main_state_history_ring[2]
    };
};
