#pragma once

#include <cstdint>

class BlackboxMessageQueueBase {
public:
    virtual int32_t WAIT_IF_EMPTY(uint32_t& timeMicroseconds) const = 0;
};
