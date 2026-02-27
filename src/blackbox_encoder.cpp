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

#include "blackbox_encoder.h"
#include "blackbox_serial_device.h"

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,readability-magic-numbers)

// !!TODO move all the printf related into Blackbox
void BlackboxEncoder::putc(void* handle, char c)
{
    reinterpret_cast<BlackboxSerialDevice*>(handle)->write(static_cast<uint8_t>(c)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
}

/*!
beginFrame and endFrame to support future Huffman compression of frame.
*/
void BlackboxEncoder::beginFrame(uint8_t value)
{
    _serial_device.write(value);
}

void BlackboxEncoder::end_frame()
{
}

void BlackboxEncoder::write(uint8_t value)
{
    _serial_device.write(value);
}

#if false
size_t BlackboxEncoder::writeString(const char *s)
{
    return _serial_device.write(reinterpret_cast<const uint8_t*>(s), strlen(s));
}
#endif

/*
size_t BlackboxEncoder::writeString(const char* s)
{
    const auto* pos = reinterpret_cast<const uint8_t*>(s); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    size_t length = 0;
    while (*pos) {
        write(*pos);
        ++pos; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ++length;
        //if (length >= _buf.max_size()) {
        //    break;
        //}
    }

    return length;
}
*/
/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
void BlackboxEncoder::writeUnsignedVB(uint32_t value)
{
    //While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127) {
        write(static_cast<uint8_t>(value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    write(static_cast<uint8_t>(value));
}


void BlackboxEncoder::writeSignedVBArray(const int32_t *array, size_t count)
{
    for (size_t ii = 0; ii < count; ++ii) {
        writeSignedVB(array[ii]);
    }
}

void BlackboxEncoder::writeSigned16VBArray(const int16_t *array, size_t count)
{
    for (size_t ii = 0; ii < count; ++ii) {
        writeSignedVB(array[ii]);
    }
}

void BlackboxEncoder::writeSigned16VBArray(const std::array<int16_t, 4>& array)
{
    for (auto item : array) {
        writeSignedVB(item);
    }
}


/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 4, 6 or 32 bits
 */
void BlackboxEncoder::writeTag2_3S32(const int32_t *values) // NOLINT(readability-function-cognitive-complexity)
{
    static const int NUM_FIELDS = 3;

    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum {
        BITS_2  = 0,
        BITS_4  = 1,
        BITS_6  = 2,
        BITS_32 = 3
    };

    enum {
        BYTES_1  = 0,
        BYTES_2  = 1,
        BYTES_3  = 2,
        BYTES_4  = 3
    };

    /*
     * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
     * below:
     *
     * Selector possibilities
     *
     * 2 bits per field  ss11 2233,
     * 4 bits per field  ss00 1111 2222 3333
     * 6 bits per field  ss11 1111 0022 2222 0033 3333
     * 32 bits per field sstt tttt followed by fields of various byte counts
     */
    int selector = BITS_2;
    for (size_t x = 0; x < NUM_FIELDS; x++) {
        //Require more than 6 bits?
        if (values[x] >= 32 || values[x] < -32) {
            selector = BITS_32;
            break;
        }

        //Require more than 4 bits?
        if (values[x] >= 8 || values[x] < -8) {
             if (selector < BITS_6) {
                 selector = BITS_6;
             }
        } else if (values[x] >= 2 || values[x] < -2) { //Require more than 2 bits?
            if (selector < BITS_4) {
                selector = BITS_4;
            }
        }
    }

    int selector2 = 0;
    switch (selector) {
    case BITS_2:
        write(static_cast<uint8_t>((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03)));
        break;
    case BITS_4:
        write(static_cast<uint8_t>((selector << 6) | (values[0] & 0x0F)));
        write(static_cast<uint8_t>((values[1] << 4) | (values[2] & 0x0F)));
        break;
    case BITS_6:
        write(static_cast<uint8_t>((selector << 6) | (values[0] & 0x3F)));
        write(static_cast<uint8_t>(values[1]));
        write(static_cast<uint8_t>(values[2]));
        break;
    case BITS_32:
        /*
         * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
         *
         * Selector2 field possibilities
         * 0 - 8 bits
         * 1 - 16 bits
         * 2 - 24 bits
         * 3 - 32 bits
         */
        selector2 = 0;

        //Encode in reverse order so the first field is in the low bits:
        for (int x = NUM_FIELDS - 1; x >= 0; --x) {
            selector2 <<= 2;

            if (values[x] < 128 && values[x] >= -128) {
                selector2 |= BYTES_1;
            } else if (values[x] < 32768 && values[x] >= -32768) {
                selector2 |= BYTES_2;
            } else if (values[x] < 8388608 && values[x] >= -8388608) {
                selector2 |= BYTES_3;
            } else {
                selector2 |= BYTES_4;
            }
        }

        //Write the selectors
        write(static_cast<uint8_t>((selector << 6) | selector2));

        //And now the values according to the selectors we picked for them
        for (size_t x = 0; x < NUM_FIELDS; ++x, selector2 >>= 2) {
            switch (selector2 & 0x03) {
            case BYTES_1:
                write(static_cast<uint8_t>(values[x]));
                break;
            case BYTES_2:
                write(static_cast<uint8_t>(values[x]));
                write(static_cast<uint8_t>(values[x] >> 8));
                break;
            case BYTES_3:
                write(static_cast<uint8_t>(values[x]));
                write(static_cast<uint8_t>(values[x] >> 8));
                write(static_cast<uint8_t>(values[x] >> 16));
                break;
            case BYTES_4:
                write(static_cast<uint8_t>(values[x]));
                write(static_cast<uint8_t>(values[x] >> 8));
                write(static_cast<uint8_t>(values[x] >> 16));
                write(static_cast<uint8_t>(values[x] >> 24));
                break;
            }
        }
        break;
    default:
        break;
    } // switch
}

/**
 * Write a 2 bit tag followed by 3 signed fields of 2, 554, 877 or 32 bits
 */
int BlackboxEncoder::writeTag2_3SVariable(const int32_t *values)
{
    static const int FIELD_COUNT = 3;
    enum {
        BITS_2  = 0,
        BITS_554  = 1,
        BITS_877  = 2,
        BITS_32 = 3
    };

    enum {
        BYTES_1  = 0,
        BYTES_2  = 1,
        BYTES_3  = 2,
        BYTES_4  = 3
    };


    /*
     * Find out how many bits the largest value requires to encode, and use it to choose one of the packing schemes
     * below:
     *
     * Selector possibilities
     *
     * 2 bits per field  ss11 2233,
     * 554 bits per field  ss11 1112 2222 3333
     * 877 bits per field  ss11 1111 1122 2222 2333 3333
     * 32 bits per field sstt tttt followed by fields of various byte counts
     */
    int selector = BITS_2;
    int selector2 = 0;
    // Require more than 877 bits?
    if (values[0] >= 256 || values[0] < -256
            || values[1] >= 128 || values[1] < -128
            || values[2] >= 128 || values[2] < -128) {
        selector = BITS_32;
   // Require more than 554 bits?
    } else if (values[0] >= 16 || values[0] < -16
            || values[1] >= 16 || values[1] < -16
            || values[2] >= 8 || values[2] < -8) {
        selector = BITS_877;
        // Require more than 2 bits?
    } else if (values[0] >= 2 || values[0] < -2
            || values[1] >= 2 || values[1] < -2
            || values[2] >= 2 || values[2] < -2) {
        selector = BITS_554;
    }

    switch (selector) {
    case BITS_2:
        write(static_cast<uint8_t>((selector << 6) | ((values[0] & 0x03) << 4) | ((values[1] & 0x03) << 2) | (values[2] & 0x03)));
        break;
    case BITS_554:
        // 554 bits per field  ss11 1112 2222 3333
        write(static_cast<uint8_t>((selector << 6) | ((values[0] & 0x1F) << 1) | ((values[1] & 0x1F) >> 4)));
        write(static_cast<uint8_t>(((values[1] & 0x0F) << 4) | (values[2] & 0x0F)));
        break;
    case BITS_877:
        // 877 bits per field  ss11 1111 1122 2222 2333 3333
        write(static_cast<uint8_t>((selector << 6) | ((values[0] & 0xFF) >> 2)));
        write(static_cast<uint8_t>(((values[0] & 0x03) << 6) | ((values[1] & 0x7F) >> 1)));
        write(static_cast<uint8_t>(((values[1] & 0x01) << 7) | (values[2] & 0x7F)));
        break;
    case BITS_32:
        /*
         * Do another round to compute a selector for each field, assuming that they are at least 8 bits each
         *
         * Selector2 field possibilities
         * 0 - 8 bits
         * 1 - 16 bits
         * 2 - 24 bits
         * 3 - 32 bits
         */
        selector2 = 0;
        //Encode in reverse order so the first field is in the low bits:
        for (int x = FIELD_COUNT - 1; x >= 0; --x) {
            selector2 <<= 2;

            if (values[x] < 128 && values[x] >= -128) {
                selector2 |= BYTES_1;
            } else if (values[x] < 32768 && values[x] >= -32768) {
                selector2 |= BYTES_2;
            } else if (values[x] < 8388608 && values[x] >= -8388608) {
                selector2 |= BYTES_3;
            } else {
                selector2 |= BYTES_4;
            }
        }

        //Write the selectors
        write(static_cast<uint8_t>((selector << 6) | selector2));

        //And now the values according to the selectors we picked for them
        for (size_t x = 0; x < FIELD_COUNT; ++x, selector2 >>= 2) {
            switch (selector2 & 0x03) {
            case BYTES_1:
                write(static_cast<uint8_t>(values[x]));
                break;
            case BYTES_2:
                write(static_cast<uint8_t>(values[x]));
                write(static_cast<uint8_t>(values[x] >> 8));
                break;
            case BYTES_3:
                write(static_cast<uint8_t>(values[x]));
                write(static_cast<uint8_t>(values[x] >> 8));
                write(static_cast<uint8_t>(values[x] >> 16));
                break;
            case BYTES_4:
                write(static_cast<uint8_t>(values[x]));
                write(static_cast<uint8_t>(values[x] >> 8));
                write(static_cast<uint8_t>(values[x] >> 16));
                write(static_cast<uint8_t>(values[x] >> 24));
                break;
            }
        }
        break;
    default:
        break;
    }
    return selector;
}

/**
 * Write an 8-bit selector followed by four signed fields of size 0, 4, 8 or 16 bits.
 */
void BlackboxEncoder::writeTag8_4S16(const int32_t *values)
{

    //Need to be enums rather than const ints if we want to switch on them (due to being C)
    enum {
        FIELD_ZERO  = 0,
        FIELD_4BIT  = 1,
        FIELD_8BIT  = 2,
        FIELD_16BIT = 3
    };

    uint8_t selector = 0;
    //Encode in reverse order so the first field is in the low bits:
    for (int x = 3; x >= 0; --x) {
        selector <<= 2;

        if (values[x] == 0) {
            selector |= FIELD_ZERO;
        } else if (values[x] < 8 && values[x] >= -8) {
            selector |= FIELD_4BIT;
        } else if (values[x] < 128 && values[x] >= -128) {
            selector |= FIELD_8BIT;
        } else {
            selector |= FIELD_16BIT;
        }
    }

    write(selector);

    int nibbleIndex = 0;
    uint8_t buffer = 0;
    for (size_t x = 0; x < 4; ++x, selector >>= 2) {
        switch (selector & 0x03) {
        case FIELD_ZERO:
            //No-op
            break;
        case FIELD_4BIT:
            if (nibbleIndex == 0) {
                //We fill high-bits first
                buffer = static_cast<uint8_t>(values[x] << 4);
                nibbleIndex = 1;
            } else {
                write(static_cast<uint8_t>(buffer | (values[x] & 0x0F)));
                nibbleIndex = 0;
            }
            break;
        case FIELD_8BIT:
            if (nibbleIndex == 0) {
                write(static_cast<uint8_t>(values[x]));
            } else {
                //Write the high bits of the value first (mask to avoid sign extension)
                write(static_cast<uint8_t>(buffer | ((values[x] >> 4) & 0x0F)));
                //Now put the leftover low bits into the top of the next buffer entry
                buffer = static_cast<uint8_t>(values[x] << 4);
            }
            break;
        case FIELD_16BIT:
            if (nibbleIndex == 0) {
                //Write high byte first
                write(static_cast<uint8_t>(values[x] >> 8));
                write(static_cast<uint8_t>(values[x]));
            } else {
                //First write the highest 4 bits
                write(static_cast<uint8_t>(buffer | ((values[x] >> 12) & 0x0F)));
                // Then the middle 8
                write(static_cast<uint8_t>(values[x] >> 4));
                //Only the smallest 4 bits are still left to write
                buffer = static_cast<uint8_t>(values[x] << 4);
            }
            break;
        }
    }
    //Anything left over to write?
    if (nibbleIndex == 1) {
        write(buffer);
    }
}

/**
 * Write `valueCount` fields from `values` to the Blackbox using signed variable byte encoding. A 1-byte header is
 * written first which specifies which fields are non-zero (so this encoding is compact when most fields are zero).
 *
 * valueCount must be 8 or less.
 */
void BlackboxEncoder::writeTag8_8SVB(const int32_t *values, size_t valueCount)
{
    if (valueCount > 0) {
        //If we're only writing one field then we can skip the header
        if (valueCount == 1) {
            writeSignedVB(values[0]);
        } else {
            //First write a one-byte header that marks which fields are non-zero
            uint8_t header = 0;

            // First field should be in low bits of header
            for (int ii = static_cast<int>(valueCount) - 1; ii >= 0; --ii) {
                header <<= 1;

                if (values[ii] != 0) {
                    header |= 0x01;
                }
            }

            write(header);

            for (size_t ii = 0; ii < valueCount; ++ii) {
                if (values[ii] != 0) {
                    writeSignedVB(values[ii]);
                }
            }
        }
    }
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,readability-magic-numbers)
