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
#include "BlackboxFieldDefinitions.h"
#include "BlackboxInterface.h"

#include <bitset>

class BlackboxSerialDevice;


class Blackbox {
public:
    Blackbox(uint32_t pidLoopTimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice) : 
        _serialDevice(serialDevice),
        _blackboxEncoder(_serialDevice),
        _callbacks(callbacks),
        targetPidLooptimeUs(pidLoopTimeUs)
        {}
public:
    /*
    * Ideally, each iteration in which we are logging headers would write a similar amount of data to the device as a
    * regular logging iteration. This way we won't hog the CPU by making a gigantic write:
    */
    enum { XYZ_AXIS_COUNT = 3 };
    enum { BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION = 64 };
    enum { FORMATTED_DATE_TIME_BUFSIZE = 30 };

    enum device_e : uint8_t {
        BLACKBOX_DEVICE_NONE = 0,
        BLACKBOX_DEVICE_FLASH = 1,
        BLACKBOX_DEVICE_SDCARD = 2,
        BLACKBOX_DEVICE_SERIAL = 3
    };

    enum mode_e : uint8_t {
        BLACKBOX_MODE_NORMAL = 0,
        BLACKBOX_MODE_MOTOR_TEST,
        BLACKBOX_MODE_ALWAYS_ON
    };

    enum sample_rate_e : uint8_t { // Sample rate is 1/(2^BlackboxSampleRate)
        BLACKBOX_RATE_ONE = 0,
        BLACKBOX_RATE_HALF,
        BLACKBOX_RATE_QUARTER,
        BLACKBOX_RATE_8TH,
        BLACKBOX_RATE_16TH
    };

    enum BlackboxState_e {
        BLACKBOX_STATE_DISABLED = 0,
        BLACKBOX_STATE_STOPPED,
        BLACKBOX_STATE_PREPARE_LOG_FILE,
        BLACKBOX_STATE_SEND_HEADER,
        BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
        BLACKBOX_STATE_SEND_GPS_H_HEADER,
        BLACKBOX_STATE_SEND_GPS_G_HEADER,
        BLACKBOX_STATE_SEND_SLOW_FIELD_HEADER,
        BLACKBOX_STATE_SEND_SYSINFO,
        BLACKBOX_STATE_CACHE_FLUSH,
        BLACKBOX_STATE_PAUSED,
        BLACKBOX_STATE_RUNNING,
        BLACKBOX_STATE_SHUTTING_DOWN,
        BLACKBOX_STATE_START_ERASE,
        BLACKBOX_STATE_ERASING,
        BLACKBOX_STATE_ERASED
    };
public:
    typedef uint32_t timeUs_t;
    typedef uint32_t timeMs_t;

    struct blackboxConfig_t {
        uint32_t fields_disabled_mask;
        sample_rate_e sample_rate;
        device_e device;
        mode_e mode;
    };
    struct start_t {
        uint16_t debugMode;
        uint8_t motorCount;
        uint8_t servoCount;
        uint8_t hasVoltageMeter;
        uint8_t hasCurrentMeter;
        uint8_t isRSSI_configured;
        uint8_t useDshotTelemetry;
        uint8_t hasBarometer;
        uint8_t hasMagnetometer;
        uint8_t hasRangefinder;
    };
    struct xmitState_t {
        uint32_t headerIndex;
        int32_t fieldIndex;
        uint32_t startTime;
    };
    struct gpsLocation_t {
        int32_t lat;                    // latitude * 1e+7
        int32_t lon;                    // longitude * 1e+7
        int32_t altCm;                  // altitude in 0.01m
    };
    // A value below 100 means great accuracy is possible with GPS satellite constellation
    struct gpsDilution_t {
        uint16_t pdop;                  // positional DOP - 3D (* 100)
        uint16_t hdop;                  // horizontal DOP - 2D (* 100)
        uint16_t vdop;                  // vertical DOP   - 1D (* 100)
    };
    // Only available on U-blox protocol
    struct gpsAccuracy_t {
        uint32_t hAcc;                  // horizontal accuracy in mm
        uint32_t vAcc;                  // vertical accuracy in mm
        uint32_t sAcc;                  // speed accuracy in mm/s
    };
    struct gpsSolutionData_t {
        uint32_t time;                  // GPS msToW
        uint32_t navIntervalMs;         // interval between nav solutions in ms
        gpsLocation_t llh;
        gpsDilution_t dop;
        gpsAccuracy_t acc;
        uint16_t speed3d;               // speed in 0.1m/s
        uint16_t groundSpeed;           // speed in 0.1m/s
        uint16_t groundCourse;          // degrees * 10
        uint8_t numSat;
    };
    struct gpsState_t {
        gpsLocation_t GPS_home;
        gpsLocation_t GPS_coord;
        uint8_t GPS_numSat;
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

    static bool isFieldEnabled(uint32_t mask, FlightLogFieldSelect_e field);
    bool isFieldEnabled(FlightLogFieldSelect_e field) const;

    void buildFieldConditionCache(const start_t& start);
    bool testFieldConditionUncached(FlightLogFieldCondition_e condition, const start_t& start) const;
    inline bool testFieldCondition(FlightLogFieldCondition_e condition) const { return _conditionCache.test(condition); }

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
    bool logEvent(FlightLogEvent_e event, const flightLogEventData_u* data); // E-frame
    void logEventArmingBeepIfNeeded(); // E-frame
    void logEventFlightModeIfNeeded(); // E-frame

    void logIteration(timeUs_t currentTimeUs, const xyz_t* gyroRPS, const xyz_t* gyroRPS_unfiltered, const xyz_t* acc);
    void advanceIterationTimers();
    void resetIterationTimers();
    void setState(BlackboxState_e newState);

    // !!TODO move following into BlackboxInterface??
    void init(const blackboxConfig_t& config);
    void start(const start_t& start); // should pass in parameters to be used in buildFieldConditionCache
    void finish();
    void endLog();
    void startInTestMode();
    void stopInTestMode();

    uint16_t getDebugMode() const { return _start.debugMode; };

    uint8_t calculateSampleRate(uint16_t pRatio) const;

    int calculatePDenom(int rateNum, int rateDenom) { return blackboxIInterval * rateNum / rateDenom; }
    uint8_t getRateDenom() { return blackboxPInterval; }
    uint16_t getPRatio() { return blackboxIInterval / blackboxPInterval; }

    bool inMotorTestMode();

// test functions
    int32_t getIInterval() const { return blackboxIInterval; }
    int32_t getPInterval() const { return blackboxPInterval; }
    int32_t getSInterval() const { return blackboxSInterval; }

protected:
    BlackboxSerialDevice& _serialDevice;
    BlackboxEncoder _blackboxEncoder;
    BlackboxCallbacksBase& _callbacks;
    size_t _motorCount { 4 };
    size_t _servoCount { 0 };
    float _motorOutputLow { 0.0F }; //!!TODO allow this to be set
    uint32_t _resetTime = 0;
    blackboxConfig_t _blackboxConfig {
        .fields_disabled_mask = FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER
            | FLIGHT_LOG_FIELD_CONDITION_BAROMETER
            | FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE
            | FLIGHT_LOG_FIELD_CONDITION_AMPERAGE_ADC
            | FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER
            | FLIGHT_LOG_FIELD_CONDITION_RSSI
            | FLIGHT_LOG_FIELD_CONDITION_DEBUG,
        .sample_rate = BLACKBOX_RATE_ONE,
        .device = BLACKBOX_DEVICE_SDCARD,
        .mode = BLACKBOX_MODE_NORMAL,
    };
    int32_t blackboxHeaderBudget {};
    // targetPidLooptimeUs is 1000 for 1kHz loop, 500 for 2kHz loop etc, targetPidLooptimeUs is rounded for short looptimes
    uint32_t targetPidLooptimeUs; // time in microseconds
    BlackboxState_e blackboxState = BLACKBOX_STATE_DISABLED;

    bool startedLoggingInTestMode = false;
    uint32_t blackboxLastArmingBeep = 0;
    uint32_t blackboxLastFlightModeFlags = 0; // New event tracking of flight modes
// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
    std::bitset<64> _conditionCache {};
    start_t _start {};

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
    xmitState_t  xmitState {};
    BlackboxState_e _cacheFlushNextState {};
//#if defined(USE_GPS)
    gpsSolutionData_t gpsSol {}; // this is a copy of the data received from the GPS
    gpsLocation_t GPS_home_llh {};
    gpsState_t gpsHistory {};
//#endif
    blackboxSlowState_t slowHistory {};
    // Keep a history of length 2, plus a buffer to store the new values into
    std::array<blackboxMainState_t, 3> _mainHistoryRing {};
    // These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
    std::array<blackboxMainState_t*, 3> blackboxHistory {
        &_mainHistoryRing[0],
        &_mainHistoryRing[1],
        &_mainHistoryRing[2]
    };
};
