#pragma once

#include "blackbox_callbacks_base.h"


class BlackboxCallbacksNull : public BlackboxCallbacksBase {
public:
    virtual void load_slow_state(blackbox_slow_state_t& slowState, blackbox_parameter_group_t& pg) override;
    virtual void load_main_state(blackbox_main_state_t& mainState, uint32_t currentTimeUs, blackbox_parameter_group_t& pg) override;
    virtual void load_gps_state(blackbox_gps_state_t& gpsState, blackbox_parameter_group_t& pg) override;

    virtual bool is_armed(blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_mode_active(blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_erase_mode_active(blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_mode_activation_condition_present(blackbox_parameter_group_t& pg) const override;
    virtual uint32_t get_arming_beep_time_microseconds(blackbox_parameter_group_t& pg) const override;
    virtual bool are_motors_running(blackbox_parameter_group_t& pg) const override;
    virtual uint32_t rc_mode_activation_mask(blackbox_parameter_group_t& pg) const override;
    virtual void beep(blackbox_parameter_group_t& pg) const override;
};
