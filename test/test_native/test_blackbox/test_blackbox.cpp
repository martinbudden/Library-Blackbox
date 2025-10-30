#include "BlackboxCallbacksNull.h"
#include "BlackboxFieldDefinitions.h"
#include "BlackboxMessageQueueBase.h"
#include "BlackboxNull.h"
#include "BlackboxSerialDeviceNull.h"

#include <array>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

class BlackboxMessageQueueNull : public BlackboxMessageQueueBase {
public:
    BlackboxMessageQueueNull() = default;

    int32_t WAIT_IF_EMPTY(uint32_t& timeMicroseconds) const override { (void)timeMicroseconds; return 0; }
};

class BlackboxTest : public BlackboxNull {
public:
    BlackboxTest(uint32_t pidLoopTimeUs, BlackboxCallbacksBase& callbacks, BlackboxMessageQueueBase& messageQueue, BlackboxSerialDevice& serialDevice) :
        BlackboxNull(pidLoopTimeUs, callbacks, messageQueue, serialDevice) {}
public:
    uint32_t getBlackboxIteration() const { return _iteration; }
    uint16_t getBlackboxLoopIndex() const { return _loopIndex; }
    uint16_t get_PFrameIndex() const { return _PFrameIndex; }

    state_e getBlackboxState() const { return _state; }
    xmit_state_t getXmitState() const { return _xmitState; }
    size_t getHeaderBudget() const { return _headerBudget; }
};

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,misc-const-correctness,readability-magic-numbers)
void test_blackbox_init()
{
    static BlackboxSerialDeviceNull serialDevice;
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);

    TEST_ASSERT_EQUAL(Blackbox::STATE_DISABLED, blackbox.getBlackboxState());

    TEST_ASSERT_EQUAL(0, blackbox.get_PFrameIndex());

    TEST_ASSERT_EQUAL(true, blackbox.shouldLogIFrame()); // IFrames are keyframes
    TEST_ASSERT_EQUAL(false, blackbox.shouldLogPFrame()); // PFrames are delta frames

    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        .mode = Blackbox::MODE_NORMAL, // logging starts immediately, file is saved when disarmed
    });

    TEST_ASSERT_EQUAL(32, blackbox.getIInterval());
    TEST_ASSERT_EQUAL(1, blackbox.getPInterval());
    TEST_ASSERT_EQUAL(8192, blackbox.getSInterval());
}

void test_blackbox_init2()
{
    static BlackboxSerialDeviceNull serialDevice;
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 5000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);

    TEST_ASSERT_EQUAL(Blackbox::STATE_DISABLED, blackbox.getBlackboxState());

    TEST_ASSERT_EQUAL(0, blackbox.get_PFrameIndex());

    TEST_ASSERT_EQUAL(true, blackbox.shouldLogIFrame()); // IFrames are keyframes
    TEST_ASSERT_EQUAL(false, blackbox.shouldLogPFrame()); // PFrames are delta frames

    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        .mode = Blackbox::MODE_NORMAL, // logging starts immediately, file is saved when disarmed
    });

    TEST_ASSERT_EQUAL(6, blackbox.getIInterval()); // every 6*5000uS = every 30ms
    TEST_ASSERT_EQUAL(1, blackbox.getPInterval());
    TEST_ASSERT_EQUAL(1536, blackbox.getSInterval()); // SFrame written every 8192ms, approx every 8 seconds
}
/*
H Product:Blackbox flight data recorder by Nicholas Sherlock
H Data version:2
H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisD[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],vbatLatest,amperageLatest,gyroADC[0],gyroADC[1],gyroADC[2],motor[0],motor[1],motor[2],motor[3]
H Field I signed:   0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1, 0,0,0,0
H Field I predictor:0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,9,0,0,0,0,11,5,5,5
H Field I encoding: 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,3,1,0,0,0, 1,0,0,0
H Field P predictor:6,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3, 3,3,3,3
H Field P encoding: 9,0,0,0,0,7,7,7,0,0,0,8,8,8,8,6,6,0,0,0, 0,0,0,0
H Field S name:flightModeFlags,stateFlags,failsafePhase,rxSignalReceived,rxFlightChannelsValid
H Field S signed:   0,0,0,0,0
H Field S predictor:0,0,0,0,0
H Field S encoding: 1,1,7,7,7
H Firmware type:Cleanflight
*/

void test_blackbox_initial_updates()
{
    static BlackboxSerialDeviceNull serialDevice;
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);
    const Blackbox::timeUs_t timeUs = 0;

    TEST_ASSERT_EQUAL(0, blackbox.getDebugMode());

    Blackbox::xmit_state_t xmitState = blackbox.getXmitState();
    TEST_ASSERT_EQUAL(0, xmitState.headerIndex);
    TEST_ASSERT_EQUAL(0, xmitState.fieldIndex);
    TEST_ASSERT_EQUAL(0, xmitState.startTime);
    TEST_ASSERT_EQUAL(0, blackbox.getHeaderBudget());


    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_SDCARD,
        .mode = Blackbox::MODE_NORMAL, // logging starts immediately, file is saved when disarmed
    });


    const Blackbox::start_t start{};
    blackbox.start(start);
    TEST_ASSERT_EQUAL(Blackbox::STATE_PREPARE_LOG_FILE, blackbox.getBlackboxState());
    xmitState = blackbox.getXmitState();
    TEST_ASSERT_EQUAL(0, xmitState.headerIndex);
    TEST_ASSERT_EQUAL(0, xmitState.fieldIndex);
    TEST_ASSERT_EQUAL(0, xmitState.startTime);
    TEST_ASSERT_EQUAL(0, blackbox.getHeaderBudget());


    blackbox.update(timeUs);
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_HEADER, blackbox.getBlackboxState());
    xmitState = blackbox.getXmitState();
    TEST_ASSERT_EQUAL(0, xmitState.headerIndex);
    TEST_ASSERT_EQUAL(0, xmitState.fieldIndex);
    TEST_ASSERT_EQUAL(0, xmitState.startTime);
    TEST_ASSERT_EQUAL(0, blackbox.getHeaderBudget());

    serialDevice.fill(0xa5);
    blackbox.update(timeUs); // write first 64 bytes of header (header length = 79)
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_HEADER, blackbox.getBlackboxState());
    xmitState = blackbox.getXmitState();
    TEST_ASSERT_EQUAL(64, xmitState.headerIndex);
    TEST_ASSERT_EQUAL(0, xmitState.startTime);
    TEST_ASSERT_EQUAL(0, blackbox.getHeaderBudget());
    // "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
    // "H Data version:2\n\0"
    TEST_ASSERT_EQUAL('H', serialDevice[0]);
    TEST_ASSERT_EQUAL(' ', serialDevice[1]);
    TEST_ASSERT_EQUAL('P', serialDevice[2]);
    TEST_ASSERT_EQUAL('r', serialDevice[3]);
    TEST_ASSERT_EQUAL('o', serialDevice[4]);
    TEST_ASSERT_EQUAL('k', serialDevice[59]);
    TEST_ASSERT_EQUAL('\n', serialDevice[60]);
    TEST_ASSERT_EQUAL('H', serialDevice[61]);
    TEST_ASSERT_EQUAL(' ', serialDevice[62]);
    TEST_ASSERT_EQUAL('D', serialDevice[63]);
    TEST_ASSERT_EQUAL(0xa5, serialDevice[64]);

    // "H Data version:2\n\0"
    blackbox.update(timeUs); // write rest of header
    xmitState = blackbox.getXmitState();
    TEST_ASSERT_EQUAL(50, blackbox.getHeaderBudget());
    TEST_ASSERT_EQUAL(0, xmitState.headerIndex);
    TEST_ASSERT_EQUAL(-1, xmitState.fieldIndex);
    TEST_ASSERT_EQUAL(0, xmitState.startTime);
    TEST_ASSERT_EQUAL('D', serialDevice[63]);
    TEST_ASSERT_EQUAL('a', serialDevice[64]);
    TEST_ASSERT_EQUAL('t', serialDevice[65]);
    TEST_ASSERT_EQUAL('a', serialDevice[66]);
    TEST_ASSERT_EQUAL(':', serialDevice[75]);
    TEST_ASSERT_EQUAL('2', serialDevice[76]);
    TEST_ASSERT_EQUAL('\n', serialDevice[77]);
    TEST_ASSERT_EQUAL(0xa5, serialDevice[78]);
    TEST_ASSERT_EQUAL(0xa5, serialDevice[79]);
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_MAIN_FIELD_HEADER, blackbox.getBlackboxState());

    serialDevice.resetIndex();
    serialDevice.fill(0xa5);
    blackbox.update(timeUs);
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_MAIN_FIELD_HEADER, blackbox.getBlackboxState());
    xmitState = blackbox.getXmitState();
    TEST_ASSERT_EQUAL(1, xmitState.headerIndex);

    // H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2],axisD[0],axisD[1],axisD[2],rcCommand[0],rcCommand[1],rcCommand[2],rcCommand[3],vbatLatest,amperageLatest,gyroADC[0],gyroADC[1],gyroADC[2],motor[0],motor[1],motor[2],motor[3]
    TEST_ASSERT_EQUAL('H', serialDevice[0]);
    TEST_ASSERT_EQUAL(' ', serialDevice[1]);
    TEST_ASSERT_EQUAL('F', serialDevice[2]);
    TEST_ASSERT_EQUAL('i', serialDevice[3]);
    TEST_ASSERT_EQUAL('e', serialDevice[4]);
    TEST_ASSERT_EQUAL('l', serialDevice[5]);
    TEST_ASSERT_EQUAL('d', serialDevice[6]);
    TEST_ASSERT_EQUAL(' ', serialDevice[7]);
    TEST_ASSERT_EQUAL('I', serialDevice[8]);
    TEST_ASSERT_EQUAL(' ', serialDevice[9]);
    TEST_ASSERT_EQUAL('n', serialDevice[10]);
    TEST_ASSERT_EQUAL('a', serialDevice[11]);
    TEST_ASSERT_EQUAL('m', serialDevice[12]);
    TEST_ASSERT_EQUAL('e', serialDevice[13]);
    TEST_ASSERT_EQUAL(':', serialDevice[14]);
    TEST_ASSERT_EQUAL('l', serialDevice[15]);
    TEST_ASSERT_EQUAL('o', serialDevice[16]);

    blackbox.update(timeUs);
    while (Blackbox::STATE_SEND_MAIN_FIELD_HEADER == blackbox.getBlackboxState()) {
        blackbox.update(timeUs);
    }

    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_SLOW_FIELD_HEADER, blackbox.getBlackboxState());
    while (Blackbox::STATE_SEND_SLOW_FIELD_HEADER == blackbox.getBlackboxState()) {
        blackbox.update(timeUs);
    }
    TEST_ASSERT_EQUAL(Blackbox::STATE_CACHE_FLUSH, blackbox.getBlackboxState());

    blackbox.update(timeUs);
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_SYSINFO, blackbox.getBlackboxState());
    while (Blackbox::STATE_SEND_SYSINFO == blackbox.getBlackboxState()) {
        blackbox.update(timeUs);
    }

    TEST_ASSERT_EQUAL(Blackbox::STATE_CACHE_FLUSH, blackbox.getBlackboxState());

    blackbox.update(timeUs);
    TEST_ASSERT_EQUAL(Blackbox::STATE_RUNNING, blackbox.getBlackboxState());

    const uint32_t iteration  = blackbox.getBlackboxIteration();
    blackbox.update(timeUs);
    TEST_ASSERT_EQUAL(iteration + 1, blackbox.getBlackboxIteration());
}

void test_blackbox_frames()
{
    static BlackboxSerialDeviceNull serialDevice;
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);

    const Blackbox::start_t start{};
    blackbox.start(start);
    TEST_ASSERT_EQUAL(Blackbox::STATE_PREPARE_LOG_FILE, blackbox.getBlackboxState());

    serialDevice.resetIndex();
    blackbox.logSFrame();
    TEST_ASSERT_EQUAL('S', serialDevice[0]);

    serialDevice.resetIndex();
    blackbox.logIFrame(); // Intraframe
    TEST_ASSERT_EQUAL('I', serialDevice[0]);

    serialDevice.resetIndex();
    blackbox.logPFrame(); // Interframe
    TEST_ASSERT_EQUAL('P', serialDevice[0]);

    serialDevice.resetIndex();
    blackbox.logPFrame(); // Interframe
    TEST_ASSERT_EQUAL('P', serialDevice[0]);

    serialDevice.resetIndex();
    blackbox.setState(Blackbox::STATE_RUNNING); // for the state to running, so logEvent will write
    const Blackbox::log_event_e event = Blackbox::LOG_EVENT_DISARM;
    const Blackbox::log_event_data_u logEventData { .disarm = { .reason = 2 } };
    const bool ret = blackbox.logEvent(event, &logEventData);
    TEST_ASSERT_EQUAL(true, ret);
    TEST_ASSERT_EQUAL('E', serialDevice[0]);
    TEST_ASSERT_EQUAL(event, serialDevice[1]);
    TEST_ASSERT_EQUAL(logEventData.disarm.reason, serialDevice[2]); // NOLINT(cppcoreguidelines-pro-type-union-access)
}

void test_blackbox_slow_header()
{
    static BlackboxSerialDeviceNull serialDevice;
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);

    const Blackbox::timeUs_t currentTimeUs = 0;

    const Blackbox::start_t start{};
    blackbox.start(start);
    TEST_ASSERT_EQUAL(Blackbox::STATE_PREPARE_LOG_FILE, blackbox.getBlackboxState());

    blackbox.setState(Blackbox::STATE_SEND_SLOW_FIELD_HEADER);
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_SLOW_FIELD_HEADER, blackbox.getBlackboxState());

    blackbox.update(currentTimeUs); // 1
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_SLOW_FIELD_HEADER, blackbox.getBlackboxState());

    blackbox.update(currentTimeUs); // 2
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_SLOW_FIELD_HEADER, blackbox.getBlackboxState());

    blackbox.update(currentTimeUs); // 3
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_SLOW_FIELD_HEADER, blackbox.getBlackboxState());

    blackbox.update(currentTimeUs); // 4
    // after 4 (BLACKBOX_SIMPLE_FIELD_HEADER_COUNT) updates, state changes to STATE_CACHE_FLUSH
    TEST_ASSERT_EQUAL(Blackbox::STATE_CACHE_FLUSH, blackbox.getBlackboxState());
}

void test_blackbox_write_sys_info()
{
    static BlackboxSerialDeviceNull serialDevice;
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);
    serialDevice.fill(0xa5);
    serialDevice.resetIndex();

    TEST_ASSERT_EQUAL(BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS, serialDevice.reserveBufferSpace(64));

    blackbox.setState(Blackbox::STATE_SEND_SYSINFO);
    TEST_ASSERT_EQUAL(Blackbox::STATE_SEND_SYSINFO, blackbox.getBlackboxState());
    TEST_ASSERT_EQUAL(0, blackbox.getXmitState().headerIndex);


    blackbox.writeSystemInformation();

    TEST_ASSERT_EQUAL('H', serialDevice[0]);
    TEST_ASSERT_EQUAL(' ', serialDevice[1]);
    TEST_ASSERT_EQUAL('F', serialDevice[2]);
    TEST_ASSERT_EQUAL('i', serialDevice[3]);
    TEST_ASSERT_EQUAL('r', serialDevice[4]);
    TEST_ASSERT_EQUAL('m', serialDevice[5]);
    TEST_ASSERT_EQUAL('w', serialDevice[6]);
    TEST_ASSERT_EQUAL('a', serialDevice[7]);
    TEST_ASSERT_EQUAL('r', serialDevice[8]);
    TEST_ASSERT_EQUAL('e', serialDevice[9]);
    TEST_ASSERT_EQUAL(' ', serialDevice[10]);
    TEST_ASSERT_EQUAL('t', serialDevice[11]);
    TEST_ASSERT_EQUAL('y', serialDevice[12]);
    TEST_ASSERT_EQUAL('p', serialDevice[13]);
    TEST_ASSERT_EQUAL('e', serialDevice[14]);
    TEST_ASSERT_EQUAL(':', serialDevice[15]);
    TEST_ASSERT_EQUAL('P', serialDevice[16]);
    TEST_ASSERT_EQUAL('r', serialDevice[17]);
    TEST_ASSERT_EQUAL('o', serialDevice[18]);
    TEST_ASSERT_EQUAL('t', serialDevice[19]);
    TEST_ASSERT_EQUAL('o', serialDevice[20]);
    TEST_ASSERT_EQUAL('F', serialDevice[21]);
    TEST_ASSERT_EQUAL('l', serialDevice[22]);
    TEST_ASSERT_EQUAL('i', serialDevice[23]);
    TEST_ASSERT_EQUAL('g', serialDevice[24]);
    TEST_ASSERT_EQUAL('h', serialDevice[25]);
    TEST_ASSERT_EQUAL('t', serialDevice[26]);
    TEST_ASSERT_EQUAL('\n', serialDevice[27]);
    TEST_ASSERT_EQUAL(0xa5, serialDevice[28]);
}

void test_blackbox_print_header_line()
{
    static BlackboxSerialDeviceNull serialDevice;
    //static BlackboxEncode blackboxEncoder(serialDevice);
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);
    serialDevice.fill(0xa5);

    serialDevice.resetIndex();
    blackbox.headerPrintfHeaderLine("Firm", "S%s", "flight");
    TEST_ASSERT_EQUAL('H', serialDevice[0]);
    TEST_ASSERT_EQUAL(' ', serialDevice[1]);
    TEST_ASSERT_EQUAL('F', serialDevice[2]);
    TEST_ASSERT_EQUAL('i', serialDevice[3]);
    TEST_ASSERT_EQUAL('r', serialDevice[4]);
    TEST_ASSERT_EQUAL('m', serialDevice[5]);
    TEST_ASSERT_EQUAL(':', serialDevice[6]);
    TEST_ASSERT_EQUAL('S', serialDevice[7]);
    TEST_ASSERT_EQUAL('f', serialDevice[8]);
    TEST_ASSERT_EQUAL('l', serialDevice[9]);
    TEST_ASSERT_EQUAL('i', serialDevice[10]);
    TEST_ASSERT_EQUAL('g', serialDevice[11]);
    TEST_ASSERT_EQUAL('h', serialDevice[12]);
    TEST_ASSERT_EQUAL('t', serialDevice[13]);
    TEST_ASSERT_EQUAL('\n', serialDevice[14]);
    TEST_ASSERT_EQUAL(0xa5, serialDevice[15]);
}

void test_blackbox_printf()
{
    static BlackboxSerialDeviceNull serialDevice;
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);

    serialDevice.fill(0xa5);
    blackbox.printf("hello"); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    TEST_ASSERT_EQUAL('h', serialDevice[0]);
    TEST_ASSERT_EQUAL('e', serialDevice[1]);
    TEST_ASSERT_EQUAL('l', serialDevice[2]);
    TEST_ASSERT_EQUAL('l', serialDevice[3]);
    TEST_ASSERT_EQUAL('o', serialDevice[4]);
    TEST_ASSERT_EQUAL(0xa5, serialDevice[5]);

    serialDevice.resetIndex();
    serialDevice.fill(0xa5);
    blackbox.printf("%s, %d", "world", 3); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    TEST_ASSERT_EQUAL('w', serialDevice[0]);
    TEST_ASSERT_EQUAL('o', serialDevice[1]);
    TEST_ASSERT_EQUAL('r', serialDevice[2]);
    TEST_ASSERT_EQUAL('l', serialDevice[3]);
    TEST_ASSERT_EQUAL('d', serialDevice[4]);
    TEST_ASSERT_EQUAL(',', serialDevice[5]);
    TEST_ASSERT_EQUAL(' ', serialDevice[6]);
    TEST_ASSERT_EQUAL('3', serialDevice[7]);
    TEST_ASSERT_EQUAL(0xa5, serialDevice[8]);

    serialDevice.resetIndex();
    serialDevice.fill(0xa5);
    blackbox.printf("S:%s, C:%c", "Ab", 'I'); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    TEST_ASSERT_EQUAL('S', serialDevice[0]);
    TEST_ASSERT_EQUAL(':', serialDevice[1]);
    TEST_ASSERT_EQUAL('A', serialDevice[2]);
    TEST_ASSERT_EQUAL('b', serialDevice[3]);
    TEST_ASSERT_EQUAL(',', serialDevice[4]);
    TEST_ASSERT_EQUAL(' ', serialDevice[5]);
    TEST_ASSERT_EQUAL('C', serialDevice[6]);
    TEST_ASSERT_EQUAL(':', serialDevice[7]);
    TEST_ASSERT_EQUAL('I', serialDevice[8]);
    TEST_ASSERT_EQUAL(0xa5, serialDevice[9]);

}

void test_blackbox_header_printf()
{
    static BlackboxSerialDeviceNull serialDevice;
    //static BlackboxEncode blackboxEncoder(serialDevice);
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);

    serialDevice.resetIndex();
    const int len = blackbox.headerPrintf("S:%s", "flight"); // NOLINT(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    TEST_ASSERT_EQUAL(8, len);
    TEST_ASSERT_EQUAL('S', serialDevice[0]);
    TEST_ASSERT_EQUAL(':', serialDevice[1]);
    TEST_ASSERT_EQUAL('f', serialDevice[2]);
    TEST_ASSERT_EQUAL('l', serialDevice[3]);
    TEST_ASSERT_EQUAL('i', serialDevice[4]);
    TEST_ASSERT_EQUAL('g', serialDevice[5]);
    TEST_ASSERT_EQUAL('h', serialDevice[6]);
    TEST_ASSERT_EQUAL('t', serialDevice[7]);
    TEST_ASSERT_EQUAL(0, serialDevice[8]);
}

void test_blackbox_fields()
{
    const uint32_t enabledMask = Blackbox::LOG_SELECT_PID
        | Blackbox::LOG_SELECT_RC_COMMANDS
        | Blackbox::LOG_SELECT_SETPOINT
        | Blackbox::LOG_SELECT_GYRO
        | Blackbox::LOG_SELECT_ACCELEROMETER
        | Blackbox::LOG_SELECT_MOTOR
        | Blackbox::LOG_SELECT_GYRO_UNFILTERED;

    TEST_ASSERT_EQUAL(true, Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_PID));
    TEST_ASSERT_EQUAL(true, Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_RC_COMMANDS));
    TEST_ASSERT_EQUAL(true, Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_SETPOINT));
    TEST_ASSERT_EQUAL(false,  Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_BATTERY_VOLTAGE));
    TEST_ASSERT_EQUAL(false,  Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_MAGNETOMETER));
    TEST_ASSERT_EQUAL(false,  Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_BAROMETER));
    TEST_ASSERT_EQUAL(false,  Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_RSSI));
    TEST_ASSERT_EQUAL(true, Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_GYRO));
    TEST_ASSERT_EQUAL(true, Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_ACCELEROMETER));
    TEST_ASSERT_EQUAL(false,  Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_DEBUG));
    TEST_ASSERT_EQUAL(true, Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_MOTOR));
    TEST_ASSERT_EQUAL(false,  Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_GPS));
    TEST_ASSERT_EQUAL(false,  Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_MOTOR_RPM));
    TEST_ASSERT_EQUAL(true, Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_GYRO_UNFILTERED));
    TEST_ASSERT_EQUAL(false,  Blackbox::isFieldEnabled(enabledMask, Blackbox::LOG_SELECT_SERVO));
}

void test_blackbox_conditions()
{
    static BlackboxSerialDeviceNull serialDevice;
    //static BlackboxEncode blackboxEncoder(serialDevice);
    static BlackboxCallbacksNull callbacks {};
    static BlackboxMessageQueueNull messageQueue {};
    enum { PID_LOOP_TIME = 1000 };
    static BlackboxTest blackbox(PID_LOOP_TIME, callbacks, messageQueue, serialDevice);

    blackbox.init({
        .sample_rate = Blackbox::RATE_ONE,
        .device = Blackbox::DEVICE_NONE,
        .mode = Blackbox::MODE_NORMAL,
    });

    blackbox.start({
        .debugMode = 0,
        .motorCount = 4,
        .servoCount = 0,
    });

    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ALWAYS));

    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_2));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_3));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_4));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_5));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_6));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_7));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_8));

    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MOTOR_1_HAS_RPM));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MOTOR_2_HAS_RPM));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MOTOR_3_HAS_RPM));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MOTOR_4_HAS_RPM));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MOTOR_5_HAS_RPM));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MOTOR_6_HAS_RPM));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MOTOR_7_HAS_RPM));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MOTOR_8_HAS_RPM));

    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SERVOS));

    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_MAGNETOMETER));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BAROMETER));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_VOLTAGE));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_BATTERY_CURRENT));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RANGEFINDER));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RSSI));

    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_K));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_ROLL));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_PITCH));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_D_YAW));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_ROLL));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_PITCH));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_PID_S_YAW));

    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_RC_COMMANDS));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_SETPOINT));

    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME));

    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_GYRO_UNFILTERED));
    TEST_ASSERT_EQUAL(true, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ACC));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_ATTITUDE));
    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_DEBUG));

    TEST_ASSERT_EQUAL(false, blackbox.testFieldCondition(FLIGHT_LOG_FIELD_CONDITION_NEVER));


}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,misc-const-correctness,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_blackbox_fields);
    RUN_TEST(test_blackbox_conditions);
    RUN_TEST(test_blackbox_init);
    RUN_TEST(test_blackbox_init2);
    RUN_TEST(test_blackbox_initial_updates);
    RUN_TEST(test_blackbox_frames);
    RUN_TEST(test_blackbox_slow_header);
    RUN_TEST(test_blackbox_write_sys_info);
    RUN_TEST(test_blackbox_print_header_line);
    RUN_TEST(test_blackbox_printf);
    RUN_TEST(test_blackbox_header_printf);

    UNITY_END();
}
