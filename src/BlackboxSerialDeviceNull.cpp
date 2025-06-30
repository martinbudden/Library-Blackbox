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
    enum { HEADER_BUDGET_SIZE = 64 };
    return HEADER_BUDGET_SIZE;
}

BlackboxSerialDevice::blackboxBufferReserveStatus_e BlackboxSerialDeviceNull::reserveBufferSpace(size_t bytes)
{
    (void)bytes;
    return BLACKBOX_RESERVE_SUCCESS;
}

size_t BlackboxSerialDeviceNull::write(uint8_t value)
{
    _buf[_index++] = value;
    return 1;
}

size_t BlackboxSerialDeviceNull::write(const uint8_t* buf, size_t length)
{
    if (_index + length < _buf.max_size()) {
        memcpy(&_buf[_index], buf, length);
        return length;
    }
    return 0;
}
