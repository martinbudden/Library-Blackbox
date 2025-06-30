#pragma once

#include "Blackbox.h"


class BlackboxNull : public Blackbox {
public:
    BlackboxNull(uint32_t pidLoopTimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice) :
        Blackbox(pidLoopTimeUs, callbacks, serialDevice) {}
public:
    virtual bool writeSystemInformation() override;
};
