#include "Blackbox.h"
#include "BlackboxCallbacksNull.h"


bool BlackboxCallbacksNull::isArmed() const
{
    return true;
}

bool BlackboxCallbacksNull::areMotorsRunning() const {
    return true;
}

bool BlackboxCallbacksNull::isBlackboxRcModeActive() const
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

void BlackboxCallbacksNull::loadSlowState(blackboxSlowState_t& slow)
{
    slow.flightModeFlags = 0;
    slow.stateFlags = 0;
    slow.failsafePhase = 0;
    slow.rxSignalReceived = false;
    slow.rxFlightChannelsValid = false;
}

void BlackboxCallbacksNull::loadMainState(blackboxMainState_t& blackboxCurrent, uint32_t currentTimeUs)
{
    (void)blackboxCurrent;
    (void)currentTimeUs;
}

uint32_t BlackboxCallbacksNull::rcModeActivationMask() const
{
    return 0;
}
