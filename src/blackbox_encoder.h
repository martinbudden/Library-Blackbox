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

#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>

class BlackboxSerialDevice;

class BlackboxEncoder {
public:
    explicit BlackboxEncoder(BlackboxSerialDevice& serialDevice) :
        _serialDevice(serialDevice) {}
public:
    static void putc(void* handle, char c);

    void beginFrame(uint8_t value);
    void endFrame();

    void write(uint8_t value);

    // VB - variable byte
    void writeUnsignedVB(uint32_t value);
    static uint32_t zigzagEncode(int32_t value) { return static_cast<uint32_t>((value << 1U) ^ (value >> 31U)); }
    void writeSignedVB(int32_t value) { //!< Write a signed integer using ZigZig and variable byte encoding.
        writeUnsignedVB(zigzagEncode(value)); // ZigZag encode to make the value always positive
    }
    void writeSignedVBArray(const int32_t* array, size_t count);
    void writeSignedVBArray(const std::array<int32_t, 3>& array) { for (auto item : array) { writeSignedVB(item); } }

    void writeSigned16VBArray(const std::array<int16_t, 4>& array);
    void writeSigned16VBArray(const int16_t* array, size_t count);
    void writeS16(int16_t value) {
        write(static_cast<uint8_t>(value & 0xFFU));
        write(static_cast<uint8_t>((value >> 8U) & 0xFFU));
    }
    void writeTag2_3S32(const int32_t* values);
    int writeTag2_3SVariable(const int32_t* values);
    void writeTag8_4S16(const int32_t* values);
    void writeTag8_8SVB(const int32_t* values, size_t valueCount);

    void writeU32(uint32_t value) {
        write(static_cast<uint8_t>(value));
        write(static_cast<uint8_t>(value >> 8U));
        write(static_cast<uint8_t>(value >> 16U));
        write(static_cast<uint8_t>(value >> 24U));
    }
    void writeFloat(float value) {
        writeU32(castFloatBytesToInt(value)); // NOLINT(cppcoreguidelines-pro-type-union-access)
    }
    static uint32_t castFloatBytesToInt(float value) {
        union { float f; uint32_t i; } u{.f=value};
        return u.i;
    }

protected:
    BlackboxSerialDevice& _serialDevice;
};
