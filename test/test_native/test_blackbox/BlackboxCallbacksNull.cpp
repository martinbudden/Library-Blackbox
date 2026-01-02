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

void BlackboxCallbacksNull::loadSlowState(blackbox_slow_state_t& slow)
{
    slow.flightModeFlags = 0;
    slow.stateFlags = 0;
    slow.failsafePhase = 0;
    slow.rxSignalReceived = false;
    slow.rxFlightChannelsValid = false;
}

void BlackboxCallbacksNull::loadMainState(blackbox_main_state_t& blackboxCurrent, uint32_t currentTimeUs)
{
    (void)blackboxCurrent;
    (void)currentTimeUs;
}

uint32_t BlackboxCallbacksNull::rcModeActivationMask() const
{
    return 0;
}
