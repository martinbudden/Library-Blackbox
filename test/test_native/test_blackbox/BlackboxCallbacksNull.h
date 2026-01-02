#pragma once

#include "BlackboxCallbacksBase.h"


class BlackboxCallbacksNull : public BlackboxCallbacksBase {
public:
    virtual void loadSlowState(blackbox_slow_state_t& slowState) override;
    virtual void loadMainState(blackbox_main_state_t& blackboxCurrent, uint32_t currentTimeUs) override;

    virtual bool isArmed() const override;
    virtual bool isBlackboxModeActive() const override;
    virtual bool isBlackboxEraseModeActive() const override;
    virtual bool isBlackboxModeActivationConditionPresent() const override;
    virtual uint32_t getArmingBeepTimeMicroseconds() const override;
    virtual bool areMotorsRunning() const override;
    virtual uint32_t rcModeActivationMask() const override;
};
