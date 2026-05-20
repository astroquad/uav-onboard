#include "control/GridControlMapper.hpp"
#include "mission/AltitudePolicy.hpp"
#include "mission/GridCoordinateTracker.hpp"
#include "mission/GridMission.hpp"
#include "mission/IntersectionDecision.hpp"
#include "mission/MarkerRegistry.hpp"
#include "mission/SnakePlanner.hpp"

#include <cassert>
#include <cstdint>
#include <cmath>

namespace {

onboard::vision::VisionResult makeVision(int width = 640, int height = 480)
{
    onboard::vision::VisionResult vis;
    vis.width = width;
    vis.height = height;
    return vis;
}

void addMarker(onboard::vision::VisionResult& vis, int id, float x_norm, float y_norm)
{
    onboard::vision::MarkerObservation marker;
    marker.id = id;
    marker.center_px.x = static_cast<float>(vis.width * (0.5 + 0.5 * x_norm));
    marker.center_px.y = static_cast<float>(vis.height * (0.5 + 0.5 * y_norm));
    vis.markers.push_back(marker);
}

onboard::mission::IntersectionDecision makeIntersectionDecision(
    onboard::vision::VisionResult& vis,
    float x_norm,
    float y_norm)
{
    onboard::mission::IntersectionDecision decision;
    decision.accepted_type = onboard::vision::IntersectionType::L;
    decision.accepted_branch_mask = 0x03;
    decision.front_available = true;
    decision.center_px.x = static_cast<float>(vis.width * (0.5 + 0.5 * x_norm));
    decision.center_px.y = static_cast<float>(vis.height * y_norm);
    decision.center_y_norm = y_norm;

    vis.intersection.valid = true;
    vis.intersection.intersection_detected = true;
    vis.intersection.type = onboard::vision::IntersectionType::L;
    vis.intersection.center_px = decision.center_px;
    return decision;
}

onboard::mission::GridMissionInput makeInput(
    double now_s,
    onboard::vision::VisionResult& vis,
    const onboard::mission::IntersectionDecision& decision)
{
    onboard::mission::GridMissionInput in;
    in.now_s = now_s;
    in.timestamp_ms = static_cast<std::int64_t>(now_s * 1000.0);
    in.frame_seq = static_cast<std::uint32_t>(now_s * 100.0 + 1.0);
    in.armed = true;
    in.guided_mode = true;
    in.heartbeat_recent = true;
    in.rangefinder_m = 2.0;
    in.local_altitude_m = 2.0;
    in.local_x_m = 0.0;
    in.local_y_m = 0.0;
    in.local_velocity_xy_mps = 0.0;
    in.attitude_yaw_rad = 0.0;
    in.vision = &vis;
    in.intersection_decision = decision;
    return in;
}

} // namespace

int main()
{
    onboard::mission::GridMissionConfig config;
    config.marker_lock_yaw_delta_rad = 0.0;
    config.vertiport_marker_stable_frames = 1;
    config.vertiport_yaw_stable_frames = 1;
    config.entry_blind_clear_distance_m = 1.8;
    config.entry_blind_min_s = 0.0;
    config.entry_blind_min_frames = 0;
    config.entry_intersection_min_distance_m = 1.8;
    config.entry_center_target_y_norm = 0.55;
    config.entry_center_x_tolerance_norm = 0.08;
    config.entry_center_y_tolerance_norm = 0.06;
    config.entry_center_velocity_threshold_mps = 0.20;
    config.entry_center_stable_frames = 2;
    config.entry_center_timeout_s = 5.0;

    onboard::common::IntersectionDecisionConfig idec_config;
    onboard::mission::GridCoordinateTracker tracker(idec_config);
    onboard::mission::MarkerRegistry registry;
    onboard::mission::AltitudePolicy altitude({});
    onboard::mission::SnakePlanner snake({});
    onboard::mission::GridMission mission(
        config, &tracker, &registry, &altitude, &snake, nullptr);

    mission.start(0.0);

    auto takeoff_vis = makeVision();
    auto takeoff_decision = onboard::mission::IntersectionDecision {};
    auto in = makeInput(0.1, takeoff_vis, takeoff_decision);
    auto out = mission.update(in);
    assert(out.state == onboard::mission::GridState::MarkerLockYaw);

    auto marker_vis = makeVision();
    addMarker(marker_vis, config.vertiport_marker_id, 0.0f, 0.0f);
    auto marker_in = makeInput(0.2, marker_vis, takeoff_decision);
    marker_in.local_x_m = 0.0;
    marker_in.local_y_m = 0.0;
    out = mission.update(marker_in);
    assert(out.state == onboard::mission::GridState::EntryForward);

    auto early_vis = makeVision();
    auto early_decision = makeIntersectionDecision(early_vis, 0.0f, 0.55f);
    auto early_in = makeInput(0.3, early_vis, early_decision);
    early_in.local_x_m = 0.4;
    out = mission.update(early_in);
    assert(out.state == onboard::mission::GridState::EntryForward);
    assert(!out.origin_publish_event.has_value());

    auto seen_vis = makeVision();
    auto seen_decision = makeIntersectionDecision(seen_vis, 0.0f, 0.55f);
    auto seen_in = makeInput(0.4, seen_vis, seen_decision);
    seen_in.local_x_m = 1.7;
    out = mission.update(seen_in);
    assert(out.state == onboard::mission::GridState::EntryForward);
    assert(!out.origin_publish_event.has_value());

    auto off_center_vis = makeVision();
    auto off_center_decision = makeIntersectionDecision(off_center_vis, 0.20f, 0.55f);
    auto off_center_in = makeInput(0.5, off_center_vis, off_center_decision);
    off_center_in.local_x_m = 1.9;
    out = mission.update(off_center_in);
    assert(out.state == onboard::mission::GridState::EntryCenterOrigin);
    assert(!out.origin_publish_event.has_value());

    auto centered_vis = makeVision();
    auto centered_decision = makeIntersectionDecision(centered_vis, 0.02f, 0.55f);
    auto centered_in = makeInput(0.6, centered_vis, centered_decision);
    centered_in.local_x_m = 2.0;
    out = mission.update(centered_in);
    assert(out.state == onboard::mission::GridState::EntryCenterOrigin);
    assert(out.intent == onboard::control::GridControlIntent::IntersectionCenter);
    assert(!out.origin_publish_event.has_value());

    centered_in.now_s = 0.7;
    centered_in.timestamp_ms = 700;
    centered_in.frame_seq = 71;
    out = mission.update(centered_in);
    assert(out.state == onboard::mission::GridState::SnakeForward);
    assert(out.origin_publish_event.has_value());
    assert(out.intersections_recorded == 1);
    assert(out.current_coord.x == 0);
    assert(out.current_coord.y == 0);
    assert(out.current_heading == onboard::mission::GridHeading::North);

    onboard::mission::MarkerWindow window;
    window.configure(4, 2);
    window.push(1);
    window.push(-1);
    window.push(1);
    assert(window.bestStableId() == 1);
    window.push(2);
    assert(window.bestStableId() == -1);

    onboard::control::GridControlMapperConfig mapper_config;
    onboard::control::GridControlMapper mapper(mapper_config, nullptr);
    onboard::control::GridControlMapperInput cmin;
    cmin.intent = onboard::control::GridControlIntent::IntersectionCenter;
    cmin.intersection_valid = true;
    cmin.intersection_center_x_norm = 0.5;
    cmin.intersection_center_y_norm = 0.45;
    const auto sp = mapper.compute(cmin);
    assert(sp.vx_forward_mps > 0.0f);
    assert(sp.vy_right_mps > 0.0f);

    return 0;
}
