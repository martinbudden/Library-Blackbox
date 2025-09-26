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

#include "BlackboxSerialDeviceSDCard.h"

#if defined(FRAMEWORK_ARDUINO_ESP32)
//#define USE_PRINTF
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#endif

// #defines for debugging
//#define USE_BLACKBOX_SBUF
//#define USE_PRINTF

BlackboxSerialDeviceSDCard::BlackboxSerialDeviceSDCard(stm32_spi_pins_t pins) : // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    _pins(pins)
{
}

BlackboxSerialDeviceSDCard::BlackboxSerialDeviceSDCard(spi_pins_t pins) : // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    _pins(stm32_spi_pins_t{{0,pins.cs},{0,pins.sck},{0,pins.cipo},{0,pins.copi},{0,pins.irq}})
{
}

int32_t BlackboxSerialDeviceSDCard::init()
{
    _state = INITIAL;
#if defined(FRAMEWORK_ARDUINO_ESP32)
    SPI.begin(_pins.sck.pin, _pins.cipo.pin, _pins.copi.pin, _pins.cs.pin);

    if (!SD.begin(_pins.cs.pin, SPI, 25000000)) {
        Serial.println("SD Card MOUNT FAIL");
        return -1;
    }

    Serial.println("SD Card MOUNT Success");

    const uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
    } else {
        const uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.println("SDCard Size: " + String(cardSize) + "MB, Type: " + String(cardType) + "\r\n\r\n");
        SD.mkdir("/logs");
    }
#endif
    return 0;
}


bool BlackboxSerialDeviceSDCard::open()
{
    return true;
}

void BlackboxSerialDeviceSDCard::close()
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    _file.close();
#endif
}

bool BlackboxSerialDeviceSDCard::isDeviceFull()
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    //return (_file.available() > 0) ? false : true;
    return false;
#else
    return true;
#endif
}

size_t BlackboxSerialDeviceSDCard::write(uint8_t value)
{
#if defined(USE_BLACKBOX_SBUF)
    if (_sbuf.bytesRemaining() <= 1) {
        _sbuf.switchToReader();
#if defined(FRAMEWORK_ARDUINO_ESP32)
        _file.write(_sbuf.ptr(), _sbuf.bytesWritten());
#endif
        _sbuf.reset();
    }
    _sbuf.writeU8(value);
    return 1;

#else

#if defined(FRAMEWORK_ARDUINO_ESP32)
    return _file.write(value);
#else
    (void)value;
    return 1;
#endif

#endif
}

size_t BlackboxSerialDeviceSDCard::write(const uint8_t* buf, size_t length)
{
#if defined(USE_BLACKBOX_SBUF)
    if (_sbuf.bytesRemaining() <= length) {
        _sbuf.switchToReader();
#if defined(FRAMEWORK_ARDUINO_ESP32)
        _file.write(_sbuf.ptr(), _sbuf.bytesWritten());
#endif
        _sbuf.reset();
    }
    _sbuf.writeData(buf, length);
    return length;
#else

#if defined(FRAMEWORK_ARDUINO_ESP32)
    return _file.write(buf, length);
#else
    (void)buf;
    return length;
#endif

#endif
}

//#define LOGFILE_PREFIX "LOG"
//#define LOGFILE_SUFFIX "BFL"
static auto constexpr LOGFILE_PREFIX = "LOG";
static auto constexpr LOGFILE_SUFFIX = "BFL";

void BlackboxSerialDeviceSDCard::enumerateFiles()
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    File dir = SD.open("/logs");
    File entry = dir.openNextFile();
    while (entry) {
        if (!entry.isDirectory()) {
            //Serial.printf("Entry:%s\r\n", entry.name());
            if (strncmp(entry.name(), LOGFILE_PREFIX, strlen(LOGFILE_PREFIX)) == 0) {
                    //&& strncmp(entry.name() + 8, LOGFILE_SUFFIX, strlen(LOGFILE_SUFFIX)) == 0) {
                std::array<char, 6> logSequenceNumberString;
                memcpy(&logSequenceNumberString[0], entry.name() + 3, 5);
                logSequenceNumberString[5] = '\0';
                _largestLogFileNumber = std::max(static_cast<size_t>(atoi(&logSequenceNumberString[0])), _largestLogFileNumber);
            }
        }
        entry = dir.openNextFile();
    }
    entry.close();
    dir.close();
#endif
}

void BlackboxSerialDeviceSDCard::createLogFile()
{
    ++_largestLogFileNumber;

#if defined(FRAMEWORK_ARDUINO_ESP32)
    // filename is of form LOGnnnnn.BFL
    std::array<char, 20> filename;
    strcpy(&filename[0], "/logs/LOGnnnnn.BFL");
    const int offset = 9;
    size_t remainder = _largestLogFileNumber;
    for (int ii = 4; ii >= 0; --ii) {
        filename[ii + offset] = (remainder % 10) + '0';
        remainder /= 10;
    }

#if defined(USE_PRINTF)
    Serial.printf("SD:Blackbox opening file:%s\r\n", &filename[0]);
#endif
    _file = SD.open(&filename[0], FILE_WRITE, true);
#endif
}

/*
Begin a new log on the SDCard.
Keep calling until the function returns true (open is complete).
*/
bool BlackboxSerialDeviceSDCard::beginLog()
{
    switch (_state) {
    case INITIAL:
#if defined(USE_PRINTF)
        Serial.printf("begin:INITIAL\r\n");
#endif
        _state = ENUMERATE_FILES;
        break;

    case ENUMERATE_FILES:
#if defined(USE_PRINTF)
        Serial.printf("begin:ENUMERATE\r\n");
#endif
        enumerateFiles();
        _state = READY_TO_CREATE_LOG;
        break;
    case READY_TO_CREATE_LOG:
#if defined(USE_PRINTF)
        Serial.printf("begin:CREATE\r\n");
#endif
        createLogFile();
        _state = READY_TO_LOG;
        break;

    case READY_TO_LOG:
#if defined(USE_PRINTF)
        Serial.printf("begin:READY\r\n");
#endif
        return true; // Log has been created!
    }

    // Not finished yet
    return false;
}

bool BlackboxSerialDeviceSDCard::endLog(bool retainLog)
{
#if defined(USE_PRINTF)
    Serial.printf("SD:endLog\r\n\r\n\r\n");
#endif
    (void)retainLog;

#if defined(FRAMEWORK_ARDUINO_ESP32)
    _file.close();
#endif

    _state = INITIAL;

    return true;
}

bool BlackboxSerialDeviceSDCard::flush()
{
#if defined(FRAMEWORK_ARDUINO_ESP32)
    _file.flush();
#endif
    return true;
}

bool BlackboxSerialDeviceSDCard::flushForce()
{
    return flush();
}

bool BlackboxSerialDeviceSDCard::flushForceComplete()
{
    return flush();
}

/*
 * Ideally, each iteration in which we are logging headers would write a similar amount of data to the device as a
 * regular logging iteration. This way we won't hog the CPU by making a gigantic write:
 */
size_t BlackboxSerialDeviceSDCard::replenishHeaderBudget()
{
    //enum { BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET = 256 };
    //const int freeSpace = _file.available();
    //_headerBudget = MIN(MIN(freeSpace, _headerBudget + blackboxMaxHeaderBytesPerIteration), BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET);
    enum { BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION = 64 };
#if defined(USE_BLACKBOX_SBUF)
    if (_sbuf.bytesRemaining() <= BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) {
        _sbuf.switchToReader();
        _file.write(_sbuf.ptr(), _sbuf.bytesWritten());
        _sbuf.reset();
    }
#endif
    return BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;
}

BlackboxSerialDevice::blackboxBufferReserveStatus_e BlackboxSerialDeviceSDCard::reserveBufferSpace(size_t bytes)
{
    (void)bytes;
    return BLACKBOX_RESERVE_SUCCESS;
}
