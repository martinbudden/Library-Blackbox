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

uint32_t BlackboxCallbacksNull::getArmingBeepTimeMicroSeconds() const
{
    return 0;
}

void BlackboxCallbacksNull::loadSlowStateFromFlightController(blackboxSlowState_t& slow)
{
    slow.flightModeFlags = 0;
    slow.stateFlags = 0;
    slow.failsafePhase = 0;
    slow.rxSignalReceived = false;
    slow.rxFlightChannelsValid = false;
}

void BlackboxCallbacksNull::loadMainStateFromFlightController(blackboxMainState_t& blackboxCurrent)
{
    (void)blackboxCurrent;
}

void BlackboxCallbacksNull::loadMainStateFromFlightController(blackboxMainState_t& blackboxCurrent, const xyz_t& gyroRPS, const xyz_t& gyroRPS_unfiltered, const xyz_t& acc)
{
    (void)blackboxCurrent;
    (void)gyroRPS;
    (void)gyroRPS_unfiltered;
    (void)acc;
}

uint32_t BlackboxCallbacksNull::rcModeActivationMask() const
{
    return 0;
}
