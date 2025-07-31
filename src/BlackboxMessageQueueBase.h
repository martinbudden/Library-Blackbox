#pragma once

#include <cstdint>

class BlackboxMessageQueueBase {
public:
    virtual int32_t WAIT_IF_EMPTY(uint32_t& timeMicroSeconds) const = 0;
};
