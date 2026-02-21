#pragma once

#include "blackbox_callbacks_base.h"


class BlackboxCallbacksNull : public BlackboxCallbacksBase {
public:
    virtual void load_slow_state(blackbox_slow_state_t& slowState) override;
    virtual void load_main_state(blackbox_main_state_t& mainState, uint32_t currentTimeUs) override;
    virtual void load_gps_state(blackbox_gps_state_t& gpsState) override;

    virtual bool is_armed() const override;
    virtual bool is_blackbox_mode_active() const override;
    virtual bool is_blackbox_erase_mode_active() const override;
    virtual bool is_blackbox_mode_activation_condition_present() const override;
    virtual uint32_t get_arming_beep_time_microseconds() const override;
    virtual bool are_motors_running() const override;
    virtual uint32_t rc_mode_activation_mask() const override;
    virtual void beep() const override;
};
