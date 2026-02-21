#include "Blackbox.h"
#include "BlackboxCallbacksNull.h"


bool BlackboxCallbacksNull::isArmed() const
{
    return true;
}

bool BlackboxCallbacksNull::areMotorsRunning() const {
    return true;
}

bool BlackboxCallbacksNull::isBlackboxModeActive() const
{
    return true;
};

bool BlackboxCallbacksNull::isBlackboxEraseModeActive() const
{
    return true;
};

bool BlackboxCallbacksNull::isBlackboxModeActivationConditionPresent() const
{
    return true;
}

uint32_t BlackboxCallbacksNull::getArmingBeepTimeMicroseconds() const
{
    return 0;
}

void BlackboxCallbacksNull::loadSlowState(blackbox_slow_state_t& slowState)
{
    slowState.flightModeFlags = 0;
    slowState.stateFlags = 0;
    slowState.failsafePhase = 0;
    slowState.rxSignalReceived = false;
    slowState.rxFlightChannelsValid = false;
}

void BlackboxCallbacksNull::loadMainState(blackbox_main_state_t& mainState, uint32_t currentTimeUs)
{
    (void)mainState;
    (void)currentTimeUs;
}

void BlackboxCallbacksNull::loadGPS_State(blackbox_gps_state_t& gpsState)
{
    (void)gpsState;
}

uint32_t BlackboxCallbacksNull::rcModeActivationMask() const
{
    return 0;
}

void BlackboxCallbacksNull::beep() const
{
}
