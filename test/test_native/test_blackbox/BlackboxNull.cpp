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
Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0.
Returns true iff transmission is complete, otherwise call again later to continue transmission.
*/
bool BlackboxNull::writeSystemInformation()
{
    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    enum { LONGEST_LINE_LENGTH = 64 };
    if (_serialDevice.reserveBufferSpace(LONGEST_LINE_LENGTH) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        return false;
    }

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    switch (xmitState.headerIndex) {
        BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s",                   "Cleanflight");
        BLACKBOX_PRINT_HEADER_LINE("Firmware revision", "%s %s (%s) %s",    "BetaFlight", "3.3.1", "611bc70f8", "REVOLT");
        BLACKBOX_PRINT_HEADER_LINE("Firmware date", "%s %s",                "Mar 21 2018", "03:14:05");
        BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s",              "0000-01-01T00:00:00.000");
        BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s",                      "Null");
        BLACKBOX_PRINT_HEADER_LINE("I interval", "%d",                      blackboxIInterval);
        BLACKBOX_PRINT_HEADER_LINE("P interval", "%d",                      blackboxPInterval);
        BLACKBOX_PRINT_HEADER_LINE("P ratio", "%d",                         static_cast<uint16_t>(blackboxIInterval / blackboxPInterval));
        BLACKBOX_PRINT_HEADER_LINE("minthrottle", "%d",                     1000);
        BLACKBOX_PRINT_HEADER_LINE("maxthrottle", "%d",                     2000);
        BLACKBOX_PRINT_HEADER_LINE("gyro_scale","0x%x",                     BlackboxEncoder::castFloatBytesToInt(1.0F));
        BLACKBOX_PRINT_HEADER_LINE("motorOutput", "%d,%d",                  158,2047);
        //BLACKBOX_PRINT_HEADER_LINE("motor_kv", "%d",                        motorConfig()->kv);
        BLACKBOX_PRINT_HEADER_LINE("acc_1G", "%u",                          4096);
        BLACKBOX_PRINT_HEADER_LINE("looptime", "%d",                        125);
        BLACKBOX_PRINT_HEADER_LINE("gyro_sync_denom", "%d",                 1);
        BLACKBOX_PRINT_HEADER_LINE("pid_process_denom", "%d",               1);

        BLACKBOX_PRINT_HEADER_LINE("features", "%d",                        541130760); //0x2041'0008
        default:
            return true;
    }
// NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg)

    xmitState.headerIndex++;
    return false;
}
