#pragma once

#include "blackbox.h"


class BlackboxNull : public Blackbox {
public:
    BlackboxNull(uint32_t pidLoopTimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serialDevice) :
        Blackbox(pidLoopTimeUs, callbacks, serialDevice) {}
public:
    virtual write_e writeSystemInformation() override;
};
