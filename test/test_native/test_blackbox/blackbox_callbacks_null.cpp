#include "blackbox.h"
#include "blackbox_callbacks_null.h"


bool BlackboxCallbacksNull::is_armed(blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
}

bool BlackboxCallbacksNull::are_motors_running(blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
}

bool BlackboxCallbacksNull::is_blackbox_mode_active(blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
};

bool BlackboxCallbacksNull::is_blackbox_erase_mode_active(blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
};

bool BlackboxCallbacksNull::is_blackbox_mode_activation_condition_present(blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
}

uint32_t BlackboxCallbacksNull::get_arming_beep_time_microseconds(blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return 0;
}

void BlackboxCallbacksNull::load_slow_state(blackbox_slow_state_t& slowState, blackbox_parameter_group_t& pg)
{
    (void)pg;
    slowState.flight_mode_flags = 0;
    slowState.state_flags = 0;
    slowState.failsafe_phase = 0;
    slowState.rx_signal_received = false;
    slowState.rx_flight_channel_is_valid = false;
}

void BlackboxCallbacksNull::load_main_state(blackbox_main_state_t& mainState, uint32_t currentTimeUs, blackbox_parameter_group_t& pg)
{
    (void)mainState;
    (void)pg;
    (void)currentTimeUs;
}

void BlackboxCallbacksNull::load_gps_state(blackbox_gps_state_t& gpsState, blackbox_parameter_group_t& pg)
{
    (void)pg;
    (void)gpsState;
}

uint32_t BlackboxCallbacksNull::rc_mode_activation_mask(blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return 0;
}

void BlackboxCallbacksNull::beep(blackbox_parameter_group_t& pg) const
{
    (void)pg;
}
