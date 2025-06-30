#pragma once

/*
 * This file is part of the Blackbox library.
 *
 * The Blackbox library is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * The Blackbox library is distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * The Blackbox library is a port (modification) of the Blackbox implementation
 * in Betaflight (which itself was a port of the Cleanflight implementation).
 *
 * The original Betaflight copyright notice is included below, as per the GNU GPL
 * "keep intact all notices” requirement.
 */

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "BlackboxSerialDevice.h"
#include <StreamBuf.h>
#include <array>


class BlackboxSerialDeviceNull : public BlackboxSerialDevice {
public:
    virtual int32_t init() override;

    virtual size_t write(uint8_t value) override;
    virtual size_t write(const uint8_t* buf, size_t length) override;

    virtual bool open() override;
    virtual void close() override;
    virtual bool isDeviceFull() override;

    virtual bool beginLog() override;
    virtual bool endLog(bool retainLog) override;

    virtual bool flush() override;
    virtual bool flushForce() override;
    virtual bool flushForceComplete() override;

    virtual size_t replenishHeaderBudget() override;
    virtual blackboxBufferReserveStatus_e reserveBufferSpace(size_t bytes) override;

// debugging functions
    void resetIndex() { _index = 0; }
    void fill(uint8_t value) { _buf.fill(value); }
    uint8_t operator[](size_t index) const { return _buf[index]; }
public:
    size_t _index = 0;
    enum { BUFFER_SIZE = 1024 };
    std::array<uint8_t, BUFFER_SIZE> _buf;
    StreamBuf _sbuf = StreamBuf(&_buf[0], BUFFER_SIZE);
};