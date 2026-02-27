#pragma once

#include "blackbox_callbacks_base.h"


class BlackboxCallbacksNull : public BlackboxCallbacksBase {
public:
    virtual void load_slow_state(blackbox_slow_state_t& slow_state, const blackbox_parameter_group_t& pg) override;
    virtual void load_main_state(blackbox_main_state_t& mainState, uint32_t current_time_us, const blackbox_parameter_group_t& pg) override;
    virtual void load_gps_state(blackbox_gps_state_t& gps_state, const blackbox_parameter_group_t& pg) override;

    virtual bool is_armed(const blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_mode_active(const blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_erase_mode_active(const blackbox_parameter_group_t& pg) const override;
    virtual bool is_blackbox_mode_activation_condition_present(const blackbox_parameter_group_t& pg) const override;
    virtual uint32_t get_arming_beep_time_microseconds(const blackbox_parameter_group_t& pg) const override;
    virtual bool are_motors_running(const blackbox_parameter_group_t& pg) const override;
    virtual uint32_t rc_mode_activation_mask(const blackbox_parameter_group_t& pg) const override;
    virtual void beep(const blackbox_parameter_group_t& pg) const override;
};
