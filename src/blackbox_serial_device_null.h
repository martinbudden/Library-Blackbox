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

#include "blackbox_serial_device.h"
#include <array>
#include <stream_buf_writer.h>


class BlackboxSerialDeviceNull : public BlackboxSerialDevice {
public:
    virtual int32_t init() override;

    virtual size_t write(uint8_t value) override;
    virtual size_t write(const uint8_t* buf, size_t length) override;

    virtual bool open() override;
    virtual void close() override;
    virtual bool is_device_full() override;

    virtual void erase_all() override;
    virtual bool is_erased() override;

    virtual bool begin_log() override;
    virtual bool end_log(bool retainLog) override;

    virtual bool flush() override;
    virtual bool flush_force() override;
    virtual bool flush_force_complete() override;

    virtual size_t replenish_header_budget() override;
    virtual blackbox_buffer_reserve_status_e reserve_buffer_space(size_t bytes) override;

// debugging functions
    void resetIndex();
    void fill(uint8_t value);
    uint8_t operator[](size_t index) const { return _output[index]; }
    const uint8_t* get_buf() const { return &_buf[0]; }
    const char* get_buf_char() const { return reinterpret_cast<const char*>(&_buf[0]); }
    const uint8_t* get_output() const { return &_output[0]; }
    const char* get_output_char() const { return reinterpret_cast<const char*>(&_output[0]); }
public:
    size_t _index = 0;
    enum { HEADER_BUDGET_SIZE = 64 };
    enum { BUFFER_SIZE = 2048 };
    std::array<uint8_t, HEADER_BUDGET_SIZE> _buf;
    StreamBufWriter _sbuf = StreamBufWriter(&_buf[0], BUFFER_SIZE);
    std::array<uint8_t, BUFFER_SIZE> _output;
};
