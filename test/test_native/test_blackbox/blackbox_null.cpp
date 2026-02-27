#include "blackbox_callbacks_null.h"
#include "blackbox_null.h"
#include "blackbox_serial_device.h"
#include <cmath>


// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#ifndef BLACKBOX_PRINT_HEADER_LINE
#define BLACKBOX_PRINT_HEADER_LINE(name, format, ...) case __COUNTER__: \
                                                header_printf_header_line(name, format, __VA_ARGS__); \
                                                break;
#endif
// NOLINTEND(cppcoreguidelines-macro-usage)


/*!
Transmit a portion of the system information headers. Call the first time with _xmit_state.header_index == 0.
Returns true iff transmission is complete, otherwise call again later to continue transmission.
*/
Blackbox::write_e BlackboxNull::write_system_information(const blackbox_parameter_group_t& pg)
{
    (void)pg;

    constexpr float radiansToDegrees { static_cast<float>(180.0 / M_PI) };
    constexpr float gyroScale {radiansToDegrees * 10.0F};
    enum { PWM_TYPE_BRUSHED = 4 };
    enum { SERIALRX_TARGET_CUSTOM = 11 };

    // Make sure we have enough room in the buffer for our longest line (as of this writing, the "Firmware date" line)
    enum { LONGEST_LINE_LENGTH = 64 };
    if (_serial_device.reserve_buffer_space(LONGEST_LINE_LENGTH) != BlackboxSerialDevice::BLACKBOX_RESERVE_SUCCESS) {
        return WRITE_NOT_COMPLETE;
    }

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg)
    switch (_xmit_state.header_index) {
        BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s",                   "ProtoFlight");
        BLACKBOX_PRINT_HEADER_LINE("Firmware revision", "%s %s (%s) %s",    "ProtoFlight", "0.0.1", "-", "alpha");
        BLACKBOX_PRINT_HEADER_LINE("Firmware date", "%s %s",                "Jun 28 2025", "00:00:00");
        BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s",              "0000-01-01T00:00:00.000");
        BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s",                      "NullCraft");
        BLACKBOX_PRINT_HEADER_LINE("I interval", "%d",                      _iinterval);
        BLACKBOX_PRINT_HEADER_LINE("P interval", "%d",                      _pinterval);
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

    _xmit_state.header_index++;
        return WRITE_NOT_COMPLETE;
}
