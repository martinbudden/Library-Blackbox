#include "blackbox.h"
#include "blackbox_callbacks_null.h"


bool BlackboxCallbacksNull::is_armed(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
}

bool BlackboxCallbacksNull::are_motors_running(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
}

bool BlackboxCallbacksNull::is_blackbox_mode_active(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
};

bool BlackboxCallbacksNull::is_blackbox_erase_mode_active(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
};

bool BlackboxCallbacksNull::is_blackbox_mode_activation_condition_present(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return true;
}

uint32_t BlackboxCallbacksNull::get_arming_beep_time_microseconds(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return 0;
}

void BlackboxCallbacksNull::load_slow_state(blackbox_slow_state_t& slow_state, const blackbox_parameter_group_t& pg)
{
    (void)pg;
    slow_state.flight_mode_flags = 0;
    slow_state.state_flags = 0;
    slow_state.failsafe_phase = 0;
    slow_state.rx_signal_received = false;
    slow_state.rx_flight_channel_is_valid = false;
}

void BlackboxCallbacksNull::load_main_state(blackbox_main_state_t& mainState, uint32_t current_time_us, const blackbox_parameter_group_t& pg)
{
    (void)mainState;
    (void)pg;
    (void)current_time_us;
}

void BlackboxCallbacksNull::load_gps_state(blackbox_gps_state_t& gps_state, const blackbox_parameter_group_t& pg)
{
    (void)pg;
    (void)gps_state;
}

uint32_t BlackboxCallbacksNull::rc_mode_activation_mask(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return 0;
}

void BlackboxCallbacksNull::beep(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
}
