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
 * by Nicholas Sherlock (aka thenickdude), which has a GPLv3 license,
 * see https://github.com/thenickdude/blackbox
 */

#include <cstddef>
#include <cstdint>

class BlackboxSerialDevice {
public:
    enum blackbox_buffer_reserve_status_e {
        BLACKBOX_RESERVE_SUCCESS,
        BLACKBOX_RESERVE_TEMPORARY_FAILURE,
        BLACKBOX_RESERVE_PERMANENT_FAILURE
    };

public:
    virtual ~BlackboxSerialDevice() = default;
    virtual int32_t init() = 0;
    virtual size_t write(uint8_t value) = 0;
    virtual size_t write(const uint8_t* buf, size_t length) = 0;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isDeviceFull() = 0;
    
    virtual void eraseAll() = 0;
    virtual bool isErased() = 0;

    virtual bool beginLog() = 0;
    virtual bool endLog(bool retainLog) = 0;

    virtual bool flush() = 0;
    virtual bool flushForce() = 0;
    virtual bool flushForceComplete() = 0;

    virtual size_t replenishHeaderBudget() = 0;
    virtual blackbox_buffer_reserve_status_e reserveBufferSpace(size_t bytes) = 0;
};
