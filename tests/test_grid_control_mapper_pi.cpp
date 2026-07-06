// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

// Anti-windup PI behaviour for GridControlMapper's altitude and yaw laws.
// With the integral gains at their 0.0 default the mapper must reproduce the
// pure-P output exactly; with ki > 0 the integral must accumulate, clamp
// (anti-windup), and reset on the appropriate condition (loss of estimate for
// altitude, target change for yaw).
#include "control/GridControlMapper.hpp"
#include "control/GuidedVelocityController.hpp"

#include <cassert>
#include <cmath>

namespace {

using onboard::control::GridControlIntent;
using onboard::control::GridControlMapper;
using onboard::control::GridControlMapperConfig;
using onboard::control::GridControlMapperInput;
using onboard::control::GuidedVelocityController;
using onboard::control::GuidedVelocityControllerConfig;

GridControlMapperInput holdInput(double current, double target)
{
    GridControlMapperInput in;
    in.intent = GridControlIntent::HoldPosition;
    in.altitude_available = true;
    in.current_altitude_m = current;
    in.target_altitude_m = target;
    return in;
}

GridControlMapperInput yawInput(double current_yaw, double target_yaw)
{
    GridControlMapperInput in;
    in.intent = GridControlIntent::YawAlign;
    in.yaw_available = true;
    in.current_yaw_rad = current_yaw;
    in.target_yaw_rad = target_yaw;
    in.altitude_available = false;  // isolate yaw from the altitude term
    return in;
}

void runAltitudePureP()
{
    GuidedVelocityController line_controller{GuidedVelocityControllerConfig{}};
    GridControlMapperConfig config;
    config.altitude_kp = 0.4;
    config.altitude_ki = 0.0;  // default
    config.altitude_max_vz_mps = 0.35;
    GridControlMapper mapper(config, &line_controller);

    // err = +0.5 -> climb -> vz_down negative, magnitude kp*err = 0.2, and it
    // must NOT grow across repeated identical inputs when ki == 0.
    const auto a = mapper.compute(holdInput(2.0, 2.5));
    const auto b = mapper.compute(holdInput(2.0, 2.5));
    assert(std::abs(a.vz_down_mps - (-0.2f)) < 1e-5f);
    assert(std::abs(b.vz_down_mps - (-0.2f)) < 1e-5f);
}

void runAltitudeIntegralAndReset()
{
    GuidedVelocityController line_controller{GuidedVelocityControllerConfig{}};
    GridControlMapperConfig config;
    config.altitude_kp = 0.4;
    config.altitude_ki = 0.1;
    config.altitude_max_vz_mps = 0.35;
    GridControlMapper mapper(config, &line_controller);

    const auto t1 = mapper.compute(holdInput(2.0, 2.5));
    const auto t2 = mapper.compute(holdInput(2.0, 2.5));
    // Climb command grows (vz_down becomes more negative) as the integral winds.
    assert(t2.vz_down_mps < t1.vz_down_mps);

    for (int i = 0; i < 200; ++i) {
        mapper.compute(holdInput(2.0, 2.5));
    }
    const auto sat = mapper.compute(holdInput(2.0, 2.5));
    assert(std::abs(sat.vz_down_mps - (-0.35f)) < 1e-5f);  // clamped at max

    // Losing the altitude estimate resets the accumulator: when it returns the
    // command is back to the single-tick value, not the saturated one.
    GridControlMapperInput no_est = holdInput(2.0, 2.5);
    no_est.altitude_available = false;
    mapper.compute(no_est);
    const auto reacq = mapper.compute(holdInput(2.0, 2.5));
    assert(reacq.vz_down_mps > sat.vz_down_mps);   // smaller magnitude
    assert(reacq.vz_down_mps < -0.2f && reacq.vz_down_mps > -0.3f);
}

void runYawIntegralAndTargetReset()
{
    GuidedVelocityController line_controller{GuidedVelocityControllerConfig{}};
    GridControlMapperConfig config;
    config.yaw_align_kp = 0.5;
    config.yaw_align_ki = 0.1;
    config.max_yaw_rate_rad_s = 0.6;
    config.yaw_align_deadband_rad = 0.0349;
    GridControlMapper mapper(config, &line_controller);

    // Constant target/error: integral winds up, so the rate grows tick over tick.
    const auto y1 = mapper.compute(yawInput(0.0, 0.3));
    const auto y2 = mapper.compute(yawInput(0.0, 0.3));
    assert(y2.yaw_rate_rad_s > y1.yaw_rate_rad_s);

    // Changing the target resets the accumulator, so the first command at the
    // new target is the single-tick value err*(kp+ki) again, not the wound-up
    // multi-tick one.
    const auto fresh = mapper.compute(yawInput(0.0, 0.1));
    assert(fresh.yaw_rate_rad_s < y1.yaw_rate_rad_s);
    assert(std::abs(fresh.yaw_rate_rad_s - static_cast<float>(0.1 * (0.5 + 0.1))) < 1e-4f);
}

}  // namespace

int main()
{
    runAltitudePureP();
    runAltitudeIntegralAndReset();
    runYawIntegralAndTargetReset();
    return 0;
}
