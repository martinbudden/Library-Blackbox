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

#include "BlackboxSerialDeviceNull.h"
#include <cstring>


int32_t BlackboxSerialDeviceNull::init()
{
    return 0;
}

bool BlackboxSerialDeviceNull::open()
{
    return true;
}

void BlackboxSerialDeviceNull::close()
{
}

void BlackboxSerialDeviceNull::eraseAll()
{
}

bool BlackboxSerialDeviceNull::isErased()
{
    return true;
}

bool BlackboxSerialDeviceNull::beginLog()
{
    resetIndex();
    return true;
}

bool BlackboxSerialDeviceNull::endLog(bool retainLog) {
    (void)retainLog;
    return true;
}

bool BlackboxSerialDeviceNull::flush()
{
    return true;
}

bool BlackboxSerialDeviceNull::flushForce()
{
    return true;
}

bool BlackboxSerialDeviceNull::flushForceComplete()
{
    return true;
}

bool BlackboxSerialDeviceNull::isDeviceFull()
{
    return false;
}

size_t BlackboxSerialDeviceNull::replenishHeaderBudget()
{
    _sbuf.reset();
    return HEADER_BUDGET_SIZE;
}

BlackboxSerialDevice::blackbox_buffer_reserve_status_e BlackboxSerialDeviceNull::reserveBufferSpace(size_t bytes)
{
    (void)bytes;
    return BLACKBOX_RESERVE_SUCCESS;
}

size_t BlackboxSerialDeviceNull::write(uint8_t value)
{
    return write(&value, 1);
}

#define USE_SBUF
size_t BlackboxSerialDeviceNull::write(const uint8_t* buf, size_t length)
{
#if defined(USE_SBUF)
    if (_sbuf.bytesRemaining() < length) {
        _sbuf.reset();
    }
    if (_sbuf.bytesRemaining() < length) {
        _sbuf.writeData(buf, length);
    }
    if (_index + length < sizeof(_output)) {
        memcpy(&_output[_index], buf, length);
    }
    _index += length;
    return length;
#else
    if (_index + length < sizeof(_output)) {
        memcpy(&_output[_index], buf, length);
    }
    if (_index + length < _buf.max_size()) {
        memcpy(&_buf[_index], buf, length);
        _index += length;
        return length;
    }
#endif
    return 0;
}

void BlackboxSerialDeviceNull::resetIndex()
{
#if defined(USE_SBUF)
    _sbuf.reset();
#endif
    _index = 0;
}

void BlackboxSerialDeviceNull::fill(uint8_t value)
{
    _buf.fill(value);
    memset(&_output[0], value, sizeof(_output));
}
