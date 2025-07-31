#pragma once

#include "Blackbox.h"


class BlackboxNull : public Blackbox {
public:
    BlackboxNull(uint32_t pidLoopTimeUs, BlackboxCallbacksBase& callbacks, BlackboxMessageQueueBase& messageQueue, BlackboxSerialDevice& serialDevice) :
        Blackbox(pidLoopTimeUs, callbacks, messageQueue, serialDevice) {}
public:
    virtual write_e writeSystemInformation() override;
};
