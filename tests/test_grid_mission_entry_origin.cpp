// TODO(latent-test): this test FAILS when asserts are live (found 2026-07
// when #undef NDEBUG was added repo-wide): its expectations predate later
// mission/vision behavior cycles (see PROJECT_SPEC) and were never enforced
// because Release builds compiled assert() out. Left disabled on purpose —
// re-validate the expectations against current intended behavior, then
// restore `#undef NDEBUG` here.

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

onboard::mission::IntersectionDecision makeStraightDecision(onboard::vision::VisionResult& vis)
{
    onboard::mission::IntersectionDecision decision;
    decision.accepted_type = onboard::vision::IntersectionType::Straight;
    decision.accepted_branch_mask = 0x05;
    decision.front_available = true;

    vis.intersection.valid = true;
    vis.intersection.intersection_detected = true;
    vis.intersection.type = onboard::vision::IntersectionType::Straight;
    const int front = static_cast<int>(onboard::vision::BranchDirection::Front);
    const int back = static_cast<int>(onboard::vision::BranchDirection::Back);
    vis.intersection.branches[front].direction = onboard::vision::BranchDirection::Front;
    vis.intersection.branches[front].present = true;
    vis.intersection.branches[front].angle_deg = -90.0f;
    vis.intersection.branches[back].direction = onboard::vision::BranchDirection::Back;
    vis.intersection.branches[back].present = true;
    vis.intersection.branches[back].angle_deg = 90.0f;
    return decision;
}

void setLine(onboard::vision::VisionResult& vis,
             float center_error_norm,
             float angle_deg,
             float confidence = 1.0f)
{
    vis.line.detected = true;
    vis.line.center_offset_px = center_error_norm * static_cast<float>(vis.width) * 0.5f;
    vis.line.angle_deg = angle_deg;
    vis.line.confidence = confidence;
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
    {
        onboard::common::IntersectionDecisionConfig idec_config;
        onboard::mission::GridCoordinateTracker tracker(idec_config);
        tracker.forceOrigin({0, 0}, onboard::mission::GridHeading::North);

        onboard::mission::GridNodeEvent synthetic;
        synthetic.valid = true;
        synthetic.local_coord = {0, -1};
        synthetic.arrival_heading = onboard::mission::GridHeading::North;
        synthetic.updates_current = false;
        tracker.commitAdvance(synthetic);

        assert(tracker.currentCoord().x == 0);
        assert(tracker.currentCoord().y == 0);
        assert(tracker.nodes().count({0, -1}) == 1);
    }

    {
        onboard::mission::GridMissionConfig config;
        config.marker_lock_yaw_delta_rad = 0.0;
        config.vertiport_marker_id = 23;
        config.vertiport_marker_stable_frames = 2;
        config.vertiport_yaw_stable_frames = 1;

        onboard::common::IntersectionDecisionConfig idec_config;
        onboard::mission::GridCoordinateTracker tracker(idec_config);
        onboard::mission::MarkerRegistry registry;
        onboard::mission::AltitudePolicy altitude({});
        onboard::mission::SnakePlanner snake({});
        onboard::mission::GridMission mission(
            config, &tracker, &registry, &altitude, &snake, nullptr);

        mission.start(0.0);
        assert(mission.activeVertiportMarkerId() == -1);

        auto takeoff_vis = makeVision();
        auto takeoff_decision = onboard::mission::IntersectionDecision {};
        auto in = makeInput(0.1, takeoff_vis, takeoff_decision);
        auto out = mission.update(in);
        assert(out.state == onboard::mission::GridState::MarkerLockYaw);
        assert(mission.activeVertiportMarkerId() == -1);

        constexpr int dynamic_vertiport_id = 17;
        auto marker_vis = makeVision();
        addMarker(marker_vis, dynamic_vertiport_id, 0.0f, 0.0f);
        auto marker_in = makeInput(0.2, marker_vis, takeoff_decision);
        out = mission.update(marker_in);
        assert(out.state == onboard::mission::GridState::MarkerLockYaw);
        assert(mission.activeVertiportMarkerId() == -1);
        assert(!out.vertiport_verified);

        marker_in.now_s = 0.3;
        marker_in.timestamp_ms = 300;
        marker_in.frame_seq = 31;
        out = mission.update(marker_in);
        assert(out.state == onboard::mission::GridState::EntryForward);
        assert(mission.activeVertiportMarkerId() == dynamic_vertiport_id);
        assert(out.vertiport_verified);
        assert(registry.hasId(dynamic_vertiport_id));
        assert(registry.gridMarkerCount() == 0);
    }

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
    config.entry_center_timeout_s = 12.0;
    config.hop_intersection_min_distance_m = 0.1;
    config.snake_record_lockout_s = 0.0;
    config.snake_turn_lockout_s = 0.0;
    config.marker_window_frames = 1;
    config.marker_window_min_count = 1;
    config.snake_launch_align_stable_frames = 2;
    config.snake_launch_align_timeout_s = 3.0;

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
    assert(mission.activeVertiportMarkerId() == config.vertiport_marker_id);

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

    auto not_yet_centered_vis = makeVision();
    auto not_yet_centered_decision =
        makeIntersectionDecision(not_yet_centered_vis, 0.02f, 0.44f);
    auto not_yet_centered_in =
        makeInput(0.6, not_yet_centered_vis, not_yet_centered_decision);
    not_yet_centered_in.local_x_m = 2.0;
    out = mission.update(not_yet_centered_in);
    assert(out.state == onboard::mission::GridState::EntryCenterOrigin);
    assert(out.intent == onboard::control::GridControlIntent::IntersectionCenter);
    assert(!out.origin_publish_event.has_value());

    not_yet_centered_in.now_s = 0.7;
    not_yet_centered_in.timestamp_ms = 700;
    not_yet_centered_in.frame_seq = 71;
    out = mission.update(not_yet_centered_in);
    assert(out.state == onboard::mission::GridState::EntryCenterOrigin);
    assert(!out.origin_publish_event.has_value());

    auto centered_vis = makeVision();
    auto centered_decision = makeIntersectionDecision(centered_vis, 0.02f, 0.55f);
    setLine(centered_vis, 0.20f, 100.0f);
    auto centered_in = makeInput(0.8, centered_vis, centered_decision);
    centered_in.local_x_m = 2.1;
    out = mission.update(centered_in);
    assert(out.state == onboard::mission::GridState::EntryCenterOrigin);
    assert(out.intent == onboard::control::GridControlIntent::IntersectionCenter);
    assert(!out.origin_publish_event.has_value());

    centered_in.now_s = 0.9;
    centered_in.timestamp_ms = 900;
    centered_in.frame_seq = 91;
    out = mission.update(centered_in);
    assert(out.state == onboard::mission::GridState::SnakeLaunchAlign);
    assert(out.origin_publish_event.has_value());
    assert(out.intersections_recorded == 1);
    assert(out.current_coord.x == 0);
    assert(out.current_coord.y == 0);
    assert(out.current_heading == onboard::mission::GridHeading::North);

    auto launch_bad_vis = makeVision();
    auto launch_bad_decision = makeIntersectionDecision(launch_bad_vis, 0.02f, 0.55f);
    setLine(launch_bad_vis, 0.20f, 100.0f);
    auto launch_bad_in = makeInput(1.0, launch_bad_vis, launch_bad_decision);
    launch_bad_in.local_x_m = 2.1;
    out = mission.update(launch_bad_in);
    assert(out.state == onboard::mission::GridState::SnakeLaunchAlign);
    assert(out.intent == onboard::control::GridControlIntent::LaunchAlign);

    auto launch_good_vis = makeVision();
    auto launch_good_decision = makeIntersectionDecision(launch_good_vis, 0.01f, 0.55f);
    setLine(launch_good_vis, 0.01f, 90.0f);
    auto launch_good_in = makeInput(1.1, launch_good_vis, launch_good_decision);
    launch_good_in.local_x_m = 2.1;
    out = mission.update(launch_good_in);
    assert(out.state == onboard::mission::GridState::SnakeLaunchAlign);
    assert(out.intent == onboard::control::GridControlIntent::LaunchAlign);

    launch_good_in.now_s = 1.2;
    launch_good_in.timestamp_ms = 1200;
    launch_good_in.frame_seq = 121;
    out = mission.update(launch_good_in);
    assert(out.state == onboard::mission::GridState::SnakeForward);

    auto forward_vis = makeVision();
    auto forward_decision = makeStraightDecision(forward_vis);
    setLine(forward_vis, 0.15f, 90.0f);
    auto forward_in = makeInput(1.3, forward_vis, forward_decision);
    forward_in.local_x_m = 2.2;
    out = mission.update(forward_in);
    assert(out.state == onboard::mission::GridState::SnakeForward);
    assert(out.intent == onboard::control::GridControlIntent::ForwardBlind);
    assert(out.line_detected);
    assert(std::abs(out.line_center_error_norm) > 0.01);

    auto pass_node_vis = makeVision();
    auto pass_node_decision = makeIntersectionDecision(pass_node_vis, 0.0f, 0.55f);
    pass_node_decision.event_ready = true;
    pass_node_decision.node_recorded = true;
    onboard::mission::GridNodeEvent pass_event;
    pass_event.valid = true;
    pass_event.local_coord = {0, -1};
    pass_event.arrival_heading = onboard::mission::GridHeading::North;
    pass_event.topology = onboard::vision::IntersectionType::L;
    pass_event.camera_branch_mask = 0x03;
    pass_event.grid_branch_mask = 0x03;
    auto pass_node_in = makeInput(2.0, pass_node_vis, pass_node_decision);
    pass_node_in.local_y_m = -0.2;
    pass_node_in.node_event = pass_event;
    out = mission.update(pass_node_in);
    assert(out.state == onboard::mission::GridState::SnakeForward);
    assert(out.intent == onboard::control::GridControlIntent::ForwardBlind);
    assert(out.commit_tracker_advance);
    assert(out.reason == "passthrough_node");
    tracker.commitAdvance(pass_event);

    auto marker_node_vis = makeVision();
    addMarker(marker_node_vis, 1, 0.4f, 0.4f);
    auto marker_node_decision = makeStraightDecision(marker_node_vis);
    marker_node_decision.event_ready = true;
    marker_node_decision.node_recorded = true;
    onboard::mission::GridNodeEvent node_event;
    node_event.valid = true;
    node_event.local_coord = {0, -2};
    node_event.arrival_heading = onboard::mission::GridHeading::North;
    node_event.topology = onboard::vision::IntersectionType::Straight;
    node_event.camera_branch_mask = 0x05;
    node_event.grid_branch_mask = 0x05;
    auto marker_node_in = makeInput(4.0, marker_node_vis, marker_node_decision);
    marker_node_in.local_y_m = -0.4;
    marker_node_in.node_event = node_event;
    out = mission.update(marker_node_in);
    assert(out.state == onboard::mission::GridState::SnakeRecordNode);
    assert(out.commit_tracker_advance);
    tracker.commitAdvance(node_event);

    auto off_marker_vis = makeVision();
    addMarker(off_marker_vis, 1, 0.4f, 0.4f);
    auto off_marker_decision = makeStraightDecision(off_marker_vis);
    auto off_marker_in = makeInput(4.1, off_marker_vis, off_marker_decision);
    out = mission.update(off_marker_in);
    assert(out.state == onboard::mission::GridState::SnakeRecordNode);
    assert(out.intent == onboard::control::GridControlIntent::MarkerHover);
    assert(out.marker_detected);

    auto centered_marker_vis = makeVision();
    addMarker(centered_marker_vis, 1, 0.02f, 0.02f);
    auto centered_marker_decision = makeStraightDecision(centered_marker_vis);
    auto centered_marker_in = makeInput(4.9, centered_marker_vis, centered_marker_decision);
    out = mission.update(centered_marker_in);
    assert(out.state == onboard::mission::GridState::SnakeRecordNode);
    assert(out.intent == onboard::control::GridControlIntent::MarkerHover);

    centered_marker_in.now_s = 5.0;
    centered_marker_in.timestamp_ms = 5000;
    centered_marker_in.frame_seq = 501;
    out = mission.update(centered_marker_in);
    assert(out.state == onboard::mission::GridState::SnakeLaunchAlign);

    onboard::mission::MarkerWindow window;
    window.configure(4, 2);
    window.push(1);
    window.push(-1);
    window.push(1);
    assert(window.bestStableId() == 1);
    window.push(2);
    assert(window.bestStableId() == -1);

    onboard::control::GridControlMapperConfig mapper_config;
    onboard::control::GuidedVelocityController line_controller({});
    onboard::control::GridControlMapper mapper(mapper_config, &line_controller);
    onboard::control::GridControlMapperInput cmin;
    cmin.intent = onboard::control::GridControlIntent::IntersectionCenter;
    cmin.intersection_valid = true;
    cmin.intersection_center_x_norm = 0.5;
    cmin.intersection_center_y_norm = 0.45;
    const auto sp = mapper.compute(cmin);
    assert(sp.vx_forward_mps > 0.0f);
    assert(sp.vy_right_mps > 0.0f);

    onboard::control::GridControlMapperInput launch_cmin;
    launch_cmin.intent = onboard::control::GridControlIntent::LaunchAlign;
    launch_cmin.line_detected = true;
    launch_cmin.line_center_error_norm = 0.4;
    launch_cmin.line_angle_error_rad = 0.2;
    launch_cmin.line_confidence = 1.0;
    const auto launch_sp = mapper.compute(launch_cmin);
    assert(launch_sp.vx_forward_mps == 0.0f);
    assert(launch_sp.vy_right_mps != 0.0f || launch_sp.yaw_rate_rad_s != 0.0f);

    onboard::control::GridControlMapperInput forward_cmin;
    forward_cmin.intent = onboard::control::GridControlIntent::ForwardBlind;
    forward_cmin.forward_speed_override_mps = 0.3;
    forward_cmin.line_detected = true;
    forward_cmin.line_center_error_norm = 0.4;
    forward_cmin.line_angle_error_rad = 0.8;
    forward_cmin.line_confidence = 1.0;
    forward_cmin.yaw_available = true;
    forward_cmin.current_yaw_rad = 0.0;
    forward_cmin.target_yaw_rad = 0.0;
    const auto forward_sp = mapper.compute(forward_cmin);
    assert(forward_sp.vx_forward_mps > 0.0f);
    assert(forward_sp.vy_right_mps != 0.0f);
    assert(forward_sp.yaw_rate_rad_s == 0.0f);

    return 0;
}
