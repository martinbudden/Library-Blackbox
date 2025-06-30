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

#include "BlackboxCallbacksNull.h"
#include "BlackboxNull.h"
#include "BlackboxSerialDevice.h"
#include <cmath>


// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#ifndef BLACKBOX_PRINT_HEADER_LINE
#define BLACKBOX_PRINT_HEADER_LINE(name, format, ...) case __COUNTER__: \
                                                headerPrintfHeaderLine(name, format, __VA_ARGS__); \
                                                break;
#define BLACKBOX_PRINT_HEADER_LINE_CUSTOM(...) case __COUNTER__: \
                                                    {__VA_ARGS__}; \
                                               break;
#endif
// NOLINTEND(cppcoreguidelines-macro-usage)


/*!
Transmit a portion of the system information headers. Call the first time with _xmitState.headerIndex == 0.
Returns true iff transmission is complete, otherwise call again later to continue transmission.
*/
Blackbox::write_e BlackboxNull::writeSystemInformation()
{
    constexpr float radiansToDegrees { static_cast<float>(180.0 / M_PI) };
    constexpr float gyroScale {radiansToDegrees * 10.0F};
    enum { PWM_TYPE_BRUSHED = 4 };
    enum { SERIALRX_TARGET_CUSTOM = 11 };

    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    enum { LONGEST_LINE_LENGTH = 64 };
    if (_serialDevice.reserveBufferSpace(LONGEST_LINE_LENGTH) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        return WRITE_NOT_COMPLETE;
    }

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    switch (_xmitState.headerIndex) {
        BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s",                   "ProtoFlight");
        BLACKBOX_PRINT_HEADER_LINE("Firmware revision", "%s %s (%s) %s",    "ProtoFlight", "0.0.1", "-", "alpha");
        BLACKBOX_PRINT_HEADER_LINE("Firmware date", "%s %s",                "Jun 28 2025", "00:00:00");
        BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s",              "0000-01-01T00:00:00.000");
        BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s",                      "NullCraft");
        BLACKBOX_PRINT_HEADER_LINE("I interval", "%d",                      blackboxIInterval);
        BLACKBOX_PRINT_HEADER_LINE("P interval", "%d",                      blackboxPInterval);
        BLACKBOX_PRINT_HEADER_LINE("minthrottle", "%d",                     1000);
        BLACKBOX_PRINT_HEADER_LINE("maxthrottle", "%d",                     2000);
        BLACKBOX_PRINT_HEADER_LINE("gyro_scale","0x%x",                     BlackboxEncoder::castFloatBytesToInt(1.0F));
        BLACKBOX_PRINT_HEADER_LINE("motorOutput", "%d,%d",                  158,2047);
        BLACKBOX_PRINT_HEADER_LINE("acc_1G", "%u",                          4096);
        BLACKBOX_PRINT_HEADER_LINE("looptime", "%d",                        125);
        BLACKBOX_PRINT_HEADER_LINE("gyro_sync_denom", "%d",                 1);
        BLACKBOX_PRINT_HEADER_LINE("pid_process_denom", "%d",               1);
        BLACKBOX_PRINT_HEADER_LINE("serialrx_provider", "%d",               SERIALRX_TARGET_CUSTOM); // custom
        BLACKBOX_PRINT_HEADER_LINE("motor_pwm_protocol", "%d",              PWM_TYPE_BRUSHED);
        BLACKBOX_PRINT_HEADER_LINE("debug_mode", "%d",                      0);
        BLACKBOX_PRINT_HEADER_LINE("features", "%d",                        541130760); //0x2041'0008
        default:
            return WRITE_COMPLETE;
    }
// NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg)

    _xmitState.headerIndex++;
        return WRITE_NOT_COMPLETE;
}
