#pragma once

#include "blackbox.h"


class BlackboxNull : public Blackbox {
public:
    BlackboxNull(uint32_t pidLoopTimeUs, BlackboxCallbacksBase& callbacks, BlackboxSerialDevice& serial_device) :
        Blackbox(pidLoopTimeUs, callbacks, serial_device) {}
public:
    virtual write_e write_system_information(const blackbox_context_t& pg) override;
};
