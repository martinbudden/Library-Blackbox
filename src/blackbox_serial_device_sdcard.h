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

#include "BlackboxSerialDevice.h"
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <FS.h>
#endif
#include <stream_buf_writer.h>
#include <array>


class BlackboxSerialDeviceSDCard : public BlackboxSerialDevice {
public:
    struct spi_pins_t {
        uint8_t cs;
        uint8_t sck;
        uint8_t cipo; // RX, CIPO, MISO, POCI
        uint8_t copi; // TX, COPI, MOSI, PICO
        uint8_t irq; // interrupt pin
    };
    struct port_pin_t {
        uint8_t port;
        uint8_t pin;
    };
    struct stm32_spi_pins_t {
        port_pin_t cs;
        port_pin_t sck;
        port_pin_t cipo; // RX, CIPO, MISO, POCI
        port_pin_t copi; // TX, COPI, MOSI, PICO
        port_pin_t irq; // interrupt pin
    };
public:
    explicit BlackboxSerialDeviceSDCard(spi_pins_t pins);
    explicit BlackboxSerialDeviceSDCard(stm32_spi_pins_t pins);
public:
    virtual int32_t init() override;

    virtual size_t write(uint8_t value) override;
    virtual size_t write(const uint8_t* buf, size_t length) override;

    virtual bool open() override;
    virtual void close() override;
    virtual bool isDeviceFull() override;

public:
    void enumerateFiles();
    void createLogFile();
public:
    virtual void eraseAll() override;
    virtual bool isErased() override;

    virtual bool beginLog() override;
    virtual bool endLog(bool retainLog) override;

    virtual bool flush() override;
    virtual bool flushForce() override;
    virtual bool flushForceComplete() override;

    virtual size_t replenishHeaderBudget() override;
    virtual blackbox_buffer_reserve_status_e reserveBufferSpace(size_t bytes) override;
private:
    enum state_e {
        INITIAL,
        ENUMERATE_FILES,
        READY_TO_CREATE_LOG,
        READY_TO_LOG
    };
    state_e _state {INITIAL};
    size_t _largestLogFileNumber {0};
    stm32_spi_pins_t _pins;
    enum { BUFFER_SIZE = 256 };
    std::array<uint8_t, BUFFER_SIZE> _buf {};
    StreamBufWriter _sbuf = StreamBufWriter(&_buf[0], BUFFER_SIZE);

#if defined(FRAMEWORK_ARDUINO_ESP32)
    File _file {};
#endif
};
