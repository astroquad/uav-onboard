#include "mission/AltitudePolicy.hpp"

#include <algorithm>
#include <cmath>

namespace onboard::mission {

AltitudePolicy::AltitudePolicy(AltitudePolicyConfig config)
    : config_(config), current_setpoint_m_(config.vertiport_altitude_m)
{
}

AltitudePolicyOutput AltitudePolicy::update(const AltitudePolicyInput& input)
{
    // Detect "off the pad" once OFF_PAD_FORWARD has been requested.
    if (!off_pad_confirmed_ && input.off_pad_requested) {
        const bool range_ok = input.rangefinder_m.has_value() &&
            *input.rangefinder_m >= config_.off_pad_rangefinder_min_m;
        if (range_ok) {
            if (off_pad_hold_start_s_ < 0.0) {
                off_pad_hold_start_s_ = input.now_s;
            } else if (input.now_s - off_pad_hold_start_s_ >=
                       config_.off_pad_rangefinder_hold_s) {
                off_pad_confirmed_ = true;
            }
        } else {
            off_pad_hold_start_s_ = -1.0;
        }

        // Alternative trigger: travelled far enough horizontally from pad origin.
        if (!off_pad_confirmed_ &&
            input.distance_from_pad_m >= config_.off_pad_local_distance_m) {
            off_pad_confirmed_ = true;
        }
    }

    const double target = off_pad_confirmed_
        ? config_.cruise_altitude_m
        : config_.vertiport_altitude_m;

    // Ramp current setpoint toward target. dt is approximated by a fixed 50ms
    // since the mission loop runs at ~20Hz; the policy is robust to slack.
    constexpr double kAssumedDtS = 0.05;
    const double max_step = std::max(0.0, config_.ramp_rate_mps) * kAssumedDtS;
    if (max_step <= 0.0) {
        current_setpoint_m_ = target;
    } else {
        const double diff = target - current_setpoint_m_;
        if (std::abs(diff) <= max_step) {
            current_setpoint_m_ = target;
        } else if (diff > 0.0) {
            current_setpoint_m_ += max_step;
        } else {
            current_setpoint_m_ -= max_step;
        }
    }

    AltitudePolicyOutput out;
    out.target_altitude_m = current_setpoint_m_;
    out.off_pad_confirmed = off_pad_confirmed_;
    if (!off_pad_confirmed_) {
        phase_ = AltitudePhase::OnPad;
    } else if (std::abs(current_setpoint_m_ - config_.cruise_altitude_m) <= 0.05) {
        phase_ = AltitudePhase::Cruise;
    } else {
        phase_ = AltitudePhase::Ramping;
    }
    out.phase = phase_;
    return out;
}

void AltitudePolicy::reset()
{
    phase_ = AltitudePhase::OnPad;
    current_setpoint_m_ = config_.vertiport_altitude_m;
    off_pad_hold_start_s_ = -1.0;
    off_pad_confirmed_ = false;
}

} // namespace onboard::mission
