#pragma once

#include "BlackboxCallbacksBase.h"

enum sensors_e {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_RANGEFINDER = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPS_MAG = 1 << 6
};


struct pidAxisData_t {
    float P;
    float I;
    float D;
    float F;
    float S;

    float Sum;
};


struct pidf_t {
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint8_t S;
    uint16_t F;
};

struct pidProfile_t {
    enum { PID_ITEM_COUNT = 5 };
    pidf_t  pid[PID_ITEM_COUNT];
    uint16_t dterm_lpf1_static_hz;          // Static Dterm lowpass 1 filter cutoff value in hz
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
    uint16_t tpa_low_breakpoint;            // Breakpoint where lower TPA is deactivated

    uint16_t yaw_lowpass_hz;                // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff
    uint8_t itermWindup;                    // iterm windup threshold, percentage of pidSumLimit within which to limit iTerm
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.

    uint8_t dterm_lpf1_type;                // Filter type for dterm lowpass 1
    uint8_t angle_limit;                    // Max angle in degrees in Angle mode

    uint8_t tpa_mode;                       // Controls which PID terms TPA effects
    uint8_t tpa_rate;                       // Percent reduction in P or D at full throttle

    int8_t tpa_low_rate;                    // Percent reduction in P or D at zero throttle
    uint8_t tpa_low_always;                 // off, on - if OFF then low TPA is only active until tpa_low_breakpoint is reached the first time

    uint8_t ez_landing_threshold;           // Threshold stick position below which motor output is limited
    uint8_t ez_landing_limit;               // Maximum motor output when all sticks centred and throttle zero
    uint8_t ez_landing_speed;               // Speed below which motor output is limited
    uint8_t landing_disarm_threshold;            // Accelerometer vector delta (jerk) threshold with disarms if exceeded
};

struct rates_t {
    enum { ROLL = 0, PITCH = 1, YAW = 2, AXIS_COUNT = 3 };
    std::array<uint16_t, AXIS_COUNT> rate_limits;
    std::array<uint8_t, AXIS_COUNT> rcRates;
    std::array<uint8_t, AXIS_COUNT> rcExpo;
    std::array<uint8_t, AXIS_COUNT> rates;
    uint8_t throttleMidpoint;
    uint8_t throttleExpo;
    uint8_t throttleLimitType;
    uint8_t throttleLimitPercent; // Sets the maximum pilot commanded throttle limit
    uint8_t ratesType;
};


class BlackboxCallbacksNull : public BlackboxCallbacksBase {
public:
    virtual void loadSlowState(blackboxSlowState_t& slowState) override;
    virtual void loadMainState(blackboxMainState_t& blackboxCurrent, uint32_t currentTimeUs) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxRcModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroSeconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;

// for sysinfo
    rates_t currentControlRateProfile() const { return rates_t {}; }
    pidProfile_t getCurrentPidProfile() const { return pidProfile_t {}; }
    float getMotorOutputLow() const { return 0.0F; }
    //float getMotorOutputHigh() const {return 0.0F;}
};
