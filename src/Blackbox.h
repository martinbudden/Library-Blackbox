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

#include "BlackboxCallbacksBase.h"
#include "BlackboxEncoder.h"
#include "BlackboxInterface.h"

#include <bitset>

class BlackboxSerialDevice;
enum flight_log_field_condition_e : uint8_t;


class Blackbox {
public:
    Blackbox(uint32_t pidLoopTimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice) : 
        _serialDevice(serialDevice),
        _encoder(_serialDevice),
        _callbacks(callbacks),
        targetPidLooptimeUs(pidLoopTimeUs)
        {}
public:
    // Ideally, each iteration in which we are logging headers would write a similar amount of data to the device as a
    // regular logging iteration. This way we won't hog the CPU by making a gigantic write:

    enum { XYZ_AXIS_COUNT = 3 };
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
        LOG_SELECT_PID              = 0x0001,
        LOG_SELECT_SETPOINT         = 0x0002,
        LOG_SELECT_RC_COMMANDS      = 0x0004,
        LOG_SELECT_RSSI             = 0x0008,
        LOG_SELECT_GYRO             = 0x0010,
        LOG_SELECT_GYRO_UNFILTERED  = 0x0020,
        LOG_SELECT_ACCELEROMETER    = 0x0040,
        LOG_SELECT_MAGNETOMETER     = 0x0080,
        LOG_SELECT_MOTOR            = 0x0100,
        LOG_SELECT_MOTOR_RPM        = 0x0200,
        LOG_SELECT_SERVO            = 0x0400,
        LOG_SELECT_BATTERY_VOLTMETER= 0x0800,
        LOG_SELECT_CURRENT_METER    = 0x1000,
        LOG_SELECT_BAROMETER        = 0x2000,
        LOG_SELECT_RANGEFINDER      = 0x4000,
        LOG_SELECT_DEBUG            = 0x8000,
        LOG_SELECT_GPS             = 0x10000,
    };

    enum state_e {
        STATE_DISABLED = 0,
        STATE_STOPPED,
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
    typedef uint32_t timeUs_t;
    typedef uint32_t timeMs_t;

    struct config_t {
        sample_rate_e sample_rate;
        device_e device;
        mode_e mode;
    };
    struct start_t {
        uint16_t debugMode;
        uint8_t motorCount;
        uint8_t servoCount;
    };
    struct xmit_state_t {
        uint32_t headerIndex;
        int32_t fieldIndex;
        uint32_t startTime;
    };
    struct log_event_syncBeep_t {
        uint32_t time;
    };
    struct log_event_disarm_t {
        uint32_t reason;
    };
    struct log_event_flightMode_t { // New Event Data type
        uint32_t flags;
        uint32_t lastFlags;
    };
    struct log_event_inflightAdjustment_t {
        int32_t newValue;
        float newFloatValue;
        uint8_t adjustmentFunction;
        bool floatFlag;
    };
    struct log_event_loggingResume_t {
        uint32_t logIteration;
        uint32_t currentTime;
    };
    union log_event_data_u {
        log_event_syncBeep_t syncBeep;
        log_event_flightMode_t flightMode; // New event data
        log_event_disarm_t disarm;
        log_event_inflightAdjustment_t inflightAdjustment;
        log_event_loggingResume_t loggingResume;
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
        int32_t latitude;   // latitude * 1e+7
        int32_t longitude;  // longitude * 1e+7
        int32_t altitudeCm; // altitude in 0.01m
    };
    // A value below 100 means great accuracy is possible with GPS satellite constellation
    struct gps_dilution_t {
        uint16_t positionalDOP; // positional DOP - 3D (* 100)
        uint16_t horizontalDOP; // horizontal DOP - 2D (* 100)
        uint16_t verticalDOP;   // vertical DOP   - 1D (* 100)
    };
    // Only available on U-blox protocol
    struct gps_accuracy_t {
        uint32_t horizontalAccuracyMm;
        uint32_t verticalAccuracyMm;
        uint32_t speedAccuracyMmPS;
    };
    struct gps_solution_data_t {
        uint32_t time;              // GPS msToW
        uint32_t navIntervalMs;     // interval between nav solutions in ms
        gps_location_t location;
        gps_dilution_t dilution;
        gps_accuracy_t accuracy;
        uint16_t speed3d;           // speed in 0.1m/s
        uint16_t groundSpeed;       // speed in 0.1m/s
        uint16_t groundCourse;      // degrees * 10
        uint8_t satelliteCount;
    };
    struct gps_state_t {
        gps_location_t home;
        gps_location_t GPS_coord;
        uint8_t satelliteCount;
    };
public:
    enum write_e { WRITE_COMPLETE, WRITE_NOT_COMPLETE };
    virtual write_e writeSystemInformation() = 0;
    virtual uint32_t update(uint32_t currentTimeUs); // main loop function
    virtual uint32_t update(uint32_t currentTimeUs, const xyz_t* gyroRPS, const xyz_t* gyroRPS_unfiltered, const xyz_t* acc);

    bool headerReserveBufferSpace();
    int printfv(const char* fmt, va_list va);
    int printf(const char* fmt, ...);
    size_t headerPrintfHeaderLine(const char* name, const char* fmt, ...);
    size_t headerPrintf(const char* fmt, ...);
    void headerWrite(uint8_t value);
    size_t headerWriteString(const char* s);

    write_e writeHeader();
    write_e writeFieldHeaderMain();
    write_e writeFieldHeaderSlow();
    write_e writeFieldHeaderGPS_H();
    write_e writeFieldHeaderGPS_G();

    static inline bool isFieldEnabled(uint32_t enabledMask, log_field_select_e field) { return (enabledMask & field) != 0; }
    inline bool isFieldEnabled(log_field_select_e field) const { return isFieldEnabled(_logSelectEnabled, field); }

    void buildFieldConditionCache();
    bool testFieldConditionUncached(flight_log_field_condition_e condition) const;
    inline bool testFieldCondition(flight_log_field_condition_e condition) const { return _conditionCache.test(condition); }

    bool isOnlyLoggingIFrames() const { return blackboxPInterval == 0; }
    bool shouldLogPFrame() const { return blackboxPFrameIndex == 0 && blackboxPInterval != 0; }
    bool shouldLogIFrame() const { return blackboxLoopIndex == 0; }
    bool shouldLogHFrame() const;

    void logIFrame(); // Intraframe, keyframe
    void logPFrame(); // Interframe, delta frame
    void logSFrame(); // Slow frame
    bool logSFrameIfNeeded();
    void logHFrame(); // GPS home frame
    void logGFrame(timeUs_t currentTimeUs); // GPS frame
    bool logEvent(log_event_e event, const log_event_data_u* data); // E-frame
    void logEventArmingBeepIfNeeded(); // E-frame
    void logEventFlightModeIfNeeded(); // E-frame

    void logIteration(timeUs_t currentTimeUs, const xyz_t* gyroRPS, const xyz_t* gyroRPS_unfiltered, const xyz_t* acc);
    void advanceIterationTimers();
    void resetIterationTimers();
    void setState(state_e newState);

    // !!TODO move following into BlackboxInterface??
    void init(const config_t& config);
    void start(const start_t& startParameters, uint32_t logSelectEnabled);
    void start(const start_t& startParameters);
    void start();
    void finish();
    void endLog();
    void startInTestMode();
    void stopInTestMode();

    uint16_t getDebugMode() const { return _debugMode; };

    uint8_t calculateSampleRate(uint16_t pRatio) const;

    bool inMotorTestMode();

// test functions
    int32_t getIInterval() const { return blackboxIInterval; }
    int32_t getPInterval() const { return blackboxPInterval; }
    int32_t getSInterval() const { return blackboxSInterval; }
protected:
    BlackboxSerialDevice& _serialDevice;
    BlackboxEncoder _encoder;
    BlackboxCallbacksBase& _callbacks;
    size_t _motorCount;
    size_t _servoCount;
    uint32_t _logSelectEnabled {};
    float _motorOutputLow { 0.0F }; //!!TODO allow this to be set
    uint32_t _resetTime = 0;
    uint16_t _debugMode;

    config_t _config {
        .sample_rate = RATE_ONE,
        .device = DEVICE_SDCARD,
        .mode = MODE_NORMAL,
    };
    int32_t blackboxHeaderBudget {};
    // targetPidLooptimeUs is 1000 for 1kHz loop, 500 for 2kHz loop etc, targetPidLooptimeUs is rounded for short looptimes
    uint32_t targetPidLooptimeUs; // time in microseconds
    state_e _state = STATE_DISABLED;

    bool startedLoggingInTestMode = false;
    uint32_t blackboxLastArmingBeep = 0;
    uint32_t blackboxLastFlightModeFlags = 0; // New event tracking of flight modes
// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
    std::bitset<64> _conditionCache {};

    uint32_t blackboxIteration {};
    int32_t blackboxLoopIndex {};
    int32_t blackboxPFrameIndex {};
    int32_t blackboxIFrameIndex {};
    int32_t blackboxIInterval = 0; //!< number of flight loop iterations before logging I-frame, typically 32 for 1kHz loop, 64 for 2kHz loop etc
    int32_t blackboxPInterval = 0; //!< number of flight loop iterations before logging P-frame
    int32_t blackboxSInterval = 0;
    int32_t blackboxSlowFrameIterationTimer {};
    bool blackboxLoggedAnyFrames {};
    // We store voltages in I-frames relative to vbatReference, which was the voltage when the blackbox was activated.
    // This helps out since the voltage is only expected to fall from that point and we can reduce our diffs to encode.
    uint16_t vbatReference {};
    xmit_state_t  _xmitState {};
    state_e _cacheFlushNextState {};
#if defined(USE_GPS)
    gps_solution_data_t _gpsSolutionData {}; // this is a copy of the data received from the GPS
    gps_location_t _gpsHomeLocation {};
    gps_state_t _gpsState {};
#endif
    blackboxSlowState_t _slowState {};
    // Keep a history of length 2, plus a buffer to store the new values into
    std::array<blackboxMainState_t, 3> _mainStateHistoryRing {};
    // These point into _mainStateHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
    std::array<blackboxMainState_t*, 3> _mainStateHistory {
        &_mainStateHistoryRing[0],
        &_mainStateHistoryRing[1],
        &_mainStateHistoryRing[2]
    };
};
