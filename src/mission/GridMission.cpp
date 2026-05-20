#include "mission/GridMission.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace onboard::mission {
namespace {

constexpr int kFrontBranchIndex = 0;

bool nodeJustRecorded(const IntersectionDecision& decision)
{
    return decision.event_ready && decision.node_recorded;
}

bool forwardBranchPresent(const IntersectionDecision& decision)
{
    return decision.front_available;
}

bool isGridNodeType(onboard::vision::IntersectionType type)
{
    return type == onboard::vision::IntersectionType::L ||
           type == onboard::vision::IntersectionType::T ||
           type == onboard::vision::IntersectionType::Cross;
}

GridHeading turnRight(GridHeading h)
{
    switch (h) {
    case GridHeading::North: return GridHeading::East;
    case GridHeading::East:  return GridHeading::South;
    case GridHeading::South: return GridHeading::West;
    case GridHeading::West:  return GridHeading::North;
    default: return GridHeading::Unknown;
    }
}

GridHeading turnLeft(GridHeading h)
{
    switch (h) {
    case GridHeading::North: return GridHeading::West;
    case GridHeading::East:  return GridHeading::North;
    case GridHeading::South: return GridHeading::East;
    case GridHeading::West:  return GridHeading::South;
    default: return GridHeading::Unknown;
    }
}

} // namespace

const char* gridStateName(GridState state)
{
    switch (state) {
    case GridState::Idle:                 return "IDLE";
    case GridState::ArmTakeoff:           return "ARM_TAKEOFF";
    case GridState::MarkerLockYaw:        return "MARKER_LOCK_YAW";
    case GridState::EntryForward:         return "ENTRY_FORWARD";
    case GridState::EntryCenterOrigin:    return "ENTRY_CENTER_ORIGIN";
    case GridState::SnakeForward:         return "SNAKE_FORWARD";
    case GridState::SnakeRecordNode:      return "SNAKE_RECORD_NODE";
    case GridState::SnakeLaunchAlign:     return "SNAKE_LAUNCH_ALIGN";
    case GridState::SnakeStopAtCenter:    return "SNAKE_STOP_AT_CENTER";
    case GridState::SnakeTurn90:          return "SNAKE_TURN_90";
    case GridState::SnakeAdvanceOneCell:  return "SNAKE_ADVANCE_ONE_CELL";
    case GridState::SnakeTurn90Again:     return "SNAKE_TURN_90_AGAIN";
    case GridState::SnakeComplete:        return "SNAKE_COMPLETE";
    case GridState::Land:                 return "LAND";
    case GridState::EmergencyLand:        return "EMERGENCY_LAND";
    case GridState::Done:                 return "DONE";
    }
    return "UNKNOWN";
}

GridMission::GridMission(GridMissionConfig config,
                         GridCoordinateTracker* tracker,
                         MarkerRegistry* registry,
                         AltitudePolicy* altitude_policy,
                         SnakePlanner* snake_planner,
                         IntersectionDecisionEngine* decision_engine,
                         double node_record_y_min_default,
                         double node_record_y_max_default,
                         double node_record_y_min_line_enter)
    : config_(config),
      tracker_(tracker),
      registry_(registry),
      altitude_(altitude_policy),
      snake_(snake_planner),
      decision_engine_(decision_engine),
      node_record_y_min_default_(node_record_y_min_default),
      node_record_y_max_default_(node_record_y_max_default),
      node_record_y_min_line_enter_(node_record_y_min_line_enter)
{
}

void GridMission::start(double now_s)
{
    started_ = true;
    mission_start_s_ = now_s;
    state_entry_s_ = now_s;
    state_ = GridState::ArmTakeoff;
}

void GridMission::reset()
{
    started_ = false;
    state_ = GridState::Idle;
    intersections_recorded_ = 0;
    consecutive_boundary_failures_ = 0;
    vertiport_marker_stable_count_ = 0;
    vertiport_yaw_stable_count_ = 0;
    entry_center_stable_count_ = 0;
    entry_forward_start_frame_seq_ = 0;
    snake_stop_stable_count_ = 0;
    snake_stop_settle_entry_s_ = -1.0;
    snake_yaw_stable_count_ = 0;
    snake_launch_align_stable_count_ = 0;
    origin_latched_ = false;
    line_lost_start_s_ = -1.0;
    pending_turn_dir_ = SnakeTurnDir::Unknown;
    pending_post_turn_heading_ = GridHeading::Unknown;
    snake_turn_first_done_ = false;
    marker_hover_active_ = false;
    last_recorded_marker_id_ = -1;
    if (registry_) registry_->reset();
    if (tracker_) tracker_->resetLocalOrigin();
    if (altitude_) altitude_->reset();
    if (snake_) snake_->reset();
    last_node_local_x_.reset();
    last_node_local_y_.reset();
    hop_start_local_x_.reset();
    hop_start_local_y_.reset();
    hop_align_window_seen_ = false;
    last_safety_event_.clear();
    last_node_grid_branch_mask_ = 0;
    last_node_front_open_ = false;
    marker_window_.configure(config_.marker_window_frames,
                             config_.marker_window_min_count);
    marker_window_.clear();
    origin_published_ = false;
    snake_post_turn_blind_until_s_ = -1.0;
    if (decision_engine_) {
        decision_engine_->setNodeRecordYBand(node_record_y_min_default_,
                                             node_record_y_max_default_);
    }
}

std::size_t GridMission::stableMarkerCandidateCount() const
{
    // Cycle 16: 1 when the sliding window has a stable id, else 0.
    return marker_window_.bestStableId() >= 0 ? 1u : 0u;
}

// Cycle 16: sliding-window marker detector implementation.
void MarkerWindow::configure(int frames, int count)
{
    if (frames > 0) max_frames = frames;
    if (count > 0) min_count = count;
    while (static_cast<int>(ids.size()) > max_frames) ids.pop_front();
}

void MarkerWindow::clear()
{
    ids.clear();
}

void MarkerWindow::push(int id)
{
    ids.push_back(id);
    while (static_cast<int>(ids.size()) > max_frames) ids.pop_front();
    // Flush the window if two different real IDs share it — the camera is
    // ambiguous and we refuse to commit either marker until it stabilises.
    if (hasMultipleIds()) {
        ids.clear();
    }
}

bool MarkerWindow::hasMultipleIds() const
{
    int seen = -1;
    for (int v : ids) {
        if (v < 0) continue;
        if (seen < 0) seen = v;
        else if (v != seen) return true;
    }
    return false;
}

int MarkerWindow::countOf(int id) const
{
    int n = 0;
    for (int v : ids) if (v == id) ++n;
    return n;
}

int MarkerWindow::bestStableId() const
{
    int best_id = -1;
    int best_count = 0;
    for (int v : ids) {
        if (v < 0) continue;
        int c = countOf(v);
        if (c > best_count) { best_count = c; best_id = v; }
    }
    return (best_count >= min_count) ? best_id : -1;
}

void GridMission::transition(GridState next, double now_s, const std::string& reason)
{
    (void)reason;
    state_ = next;
    state_entry_s_ = now_s;
}

double GridMission::wrap(double a) const
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

std::optional<onboard::vision::MarkerObservation> GridMission::findMarker(
    const onboard::vision::VisionResult* vis, int id) const
{
    if (!vis) return std::nullopt;
    for (const auto& m : vis->markers) {
        if (m.id == id) return m;
    }
    return std::nullopt;
}

void GridMission::populateLineInputs(const GridMissionInput& in, GridMissionOutput& out) const
{
    if (!in.vision) return;
    const auto& line = in.vision->line;
    out.line_detected = line.detected;
    const int width = in.vision->width > 0 ? in.vision->width : 1;
    out.line_center_error_norm = line.center_offset_px / (width * 0.5);
    // line.angle_deg is the detected line orientation in image frame.
    // A line that runs straight ahead from the camera (aligned with vehicle
    // forward) shows up vertical, i.e. ~90 deg. The controller wants the
    // *error* relative to that ideal, wrapped into [-90, 90] deg.
    constexpr double kDesiredLineAngleDeg = 90.0;
    double diff_deg = static_cast<double>(line.angle_deg) - kDesiredLineAngleDeg;
    while (diff_deg > 90.0)  diff_deg -= 180.0;
    while (diff_deg < -90.0) diff_deg += 180.0;
    out.line_angle_error_rad = diff_deg * M_PI / 180.0;
    out.line_confidence = line.confidence;

    // Cycle 16: hop-aware freeze. The line controller only runs during the
    // brief mid-cell align window (managed by the SnakeForward /
    // SnakeAdvanceOneCell handlers). Outside that window we want yaw locked
    // to the latched target, so we zero both inputs and let the mapper send
    // ForwardBlind. Even inside the align window we still gate on
    // `accepted_type == Straight` plus the Front/Back arrow head-tail check
    // — a T entering the camera mid-window must not pull yaw aside.
    //
    // The align window is checked via hopDistance() so this works the same
    // for SnakeForward, SnakeAdvanceOneCell, and any future hop-driven state.
    const double distance = hopDistance(in);
    const bool hop_window_active = inHopAlignWindow(distance);
    const bool launch_state = state_ == GridState::SnakeLaunchAlign;
    const bool snake_state =
        (state_ == GridState::SnakeForward ||
         state_ == GridState::SnakeAdvanceOneCell);

    if (launch_state) {
        if (!forwardBranchPresent(in.intersection_decision) && !last_node_front_open_) {
            out.line_angle_error_rad = 0.0;
            out.line_center_error_norm = 0.0;
            out.line_detected = false;
        }
        return;
    }

    if (!snake_state || !hop_window_active) {
        out.line_angle_error_rad = 0.0;
        out.line_center_error_norm = 0.0;
        return;
    }

    const auto t_acc = in.intersection_decision.accepted_type;
    if (t_acc != onboard::vision::IntersectionType::Straight) {
        out.line_angle_error_rad = 0.0;
        out.line_center_error_norm = 0.0;
        return;
    }

    // Head/tail arrow gate: require the Front + Back arrows to be ~180°
    // apart within ±10°.
    const auto& brs = in.vision->intersection.branches;
    const int F = static_cast<int>(onboard::vision::BranchDirection::Front);
    const int B = static_cast<int>(onboard::vision::BranchDirection::Back);
    bool head_tail_aligned = false;
    if (brs[F].present && brs[B].present) {
        float diff = std::abs(brs[F].angle_deg - brs[B].angle_deg);
        if (diff > 180.0f) diff = 360.0f - diff;  // wrap to [0,180]
        head_tail_aligned = (std::abs(diff - 180.0f) <= 10.0f);
    }
    if (!head_tail_aligned) {
        out.line_angle_error_rad = 0.0;
        out.line_center_error_norm = 0.0;
    }
}

void GridMission::populateMarkerInputs(const GridMissionInput& in,
                                       int marker_id_focus,
                                       GridMissionOutput& out) const
{
    auto m = findMarker(in.vision, marker_id_focus);
    if (!m.has_value()) {
        out.marker_detected = false;
        return;
    }
    out.marker_detected = true;
    const int width = in.vision ? in.vision->width : 1;
    const int height = in.vision ? in.vision->height : 1;
    if (width > 0 && height > 0) {
        out.marker_center_error_x_norm = (m->center_px.x - width * 0.5) / (width * 0.5);
        out.marker_center_error_y_norm = (m->center_px.y - height * 0.5) / (height * 0.5);
    }
}

double GridMission::intersectionCenterXNorm(const GridMissionInput& in) const
{
    if (!in.vision || in.vision->width <= 0) return 0.0;
    if (!in.vision->intersection.valid) return 0.0;
    if (in.intersection_decision.accepted_type == onboard::vision::IntersectionType::None) {
        return 0.0;
    }
    const double half_width = in.vision->width * 0.5;
    return (in.intersection_decision.center_px.x - half_width) / half_width;
}

bool GridMission::isEntryIntersectionCandidate(const GridMissionInput& in) const
{
    if (!in.vision || !in.vision->intersection.valid) return false;
    if (!isGridNodeType(in.intersection_decision.accepted_type)) return false;
    return in.intersection_decision.center_y_norm <= config_.entry_center_late_y_norm;
}

bool GridMission::isIntersectionCenteredForEntry(const GridMissionInput& in) const
{
    if (!isEntryIntersectionCandidate(in)) return false;
    const double x_norm = intersectionCenterXNorm(in);
    const double y_norm = in.intersection_decision.center_y_norm;
    return std::abs(x_norm) <= config_.entry_center_x_tolerance_norm &&
           std::abs(y_norm - config_.entry_center_target_y_norm) <=
               config_.entry_center_y_tolerance_norm;
}

void GridMission::latchGridOrigin(const GridMissionInput& in, GridMissionOutput& out)
{
    if (tracker_ && !origin_latched_) {
        tracker_->forceOrigin(GridCoord{0, 0}, GridHeading::North);
        origin_latched_ = true;
    }
    intersections_recorded_ = 1;
    last_node_record_s_ = in.now_s;
    if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
        last_node_local_x_ = *in.local_x_m;
        last_node_local_y_ = *in.local_y_m;
    }
    const bool valid_intersection =
        in.vision && in.vision->intersection.valid &&
        isGridNodeType(in.intersection_decision.accepted_type);
    last_node_grid_branch_mask_ = valid_intersection
        ? rotateCameraBranchMaskToGrid(
              in.intersection_decision.accepted_branch_mask,
              GridHeading::North)
        : 0;
    last_node_front_open_ =
        valid_intersection && forwardBranchPresent(in.intersection_decision);
    if (!origin_published_) {
        GridNodeEvent origin_event;
        origin_event.valid = true;
        origin_event.node_id = 1;
        origin_event.local_coord = GridCoord{0, 0};
        origin_event.topology = valid_intersection
            ? in.intersection_decision.accepted_type
            : onboard::vision::IntersectionType::Unknown;
        origin_event.arrival_heading = GridHeading::North;
        origin_event.camera_branch_mask = valid_intersection
            ? in.intersection_decision.accepted_branch_mask
            : 0;
        origin_event.grid_branch_mask = last_node_grid_branch_mask_;
        origin_event.first_node = true;
        origin_event.origin_local_only = true;
        out.origin_publish_event = origin_event;
        origin_published_ = true;
    }
    armHopStart(in);
}

bool GridMission::isHardFailsafe(const GridMissionInput& in, std::string& reason) const
{
    if (in.now_s - mission_start_s_ > config_.mission_timeout_s) {
        reason = "mission_timeout";
        return true;
    }
    if (!in.heartbeat_recent) {
        reason = "heartbeat_lost";
        return true;
    }
    // Cycle 8: prefer rangefinder (AGL) for ceiling check. LOCAL_NED z is in
    // takeoff-origin frame which jumps when the drone leaves the vertiport
    // pad, producing false ceiling trips. rangefinder is the actual height
    // above the surface below.
    const double effective_alt =
        in.rangefinder_m.value_or(in.local_altitude_m.value_or(0.0));
    const bool has_alt =
        in.rangefinder_m.has_value() || in.local_altitude_m.has_value();
    if (has_alt && effective_alt > config_.altitude_ceiling_m) {
        reason = "altitude_ceiling";
        return true;
    }
    if (intersections_recorded_ > config_.max_intersections) {
        reason = "max_intersections";
        return true;
    }
    if (in.abort_requested) {
        reason = "abort_requested";
        return true;
    }
    return false;
}

GridMissionOutput GridMission::update(const GridMissionInput& input)
{
    GridMissionOutput out;
    out.state = state_;
    out.intent = control::GridControlIntent::Idle;
    out.takeoff_altitude_m = config_.vertiport_altitude_m;
    if (input.attitude_yaw_rad.has_value()) {
        out.yaw_available = true;
        out.current_yaw_rad = *input.attitude_yaw_rad;
    }
    if (input.local_altitude_m.has_value()) {
        out.altitude_off_pad_confirmed = false;
    }

    // Universal failsafe (skip for Idle/Done)
    if (state_ != GridState::Idle &&
        state_ != GridState::Done &&
        state_ != GridState::EmergencyLand) {
        std::string fs_reason;
        if (isHardFailsafe(input, fs_reason)) {
            last_safety_event_ = fs_reason;
            out.last_safety_event = fs_reason;
            transition(GridState::EmergencyLand, input.now_s, fs_reason);
        }
    }
    if (out.last_safety_event.empty() && !last_safety_event_.empty()) {
        out.last_safety_event = last_safety_event_;
    }

    // Always-update altitude policy (so off_pad_confirmed latches once triggered)
    AltitudePolicyInput alt_in;
    alt_in.now_s = input.now_s;
    alt_in.rangefinder_m = input.rangefinder_m;
    // Cycle 16: distance_from_pad is not used for state decisions any more.
    // Leave it at the default — EntryForward is bounded by intersection
    // detection + an explicit timeout, not by LOCAL_NED displacement.
    alt_in.off_pad_requested =
        (state_ != GridState::Idle &&
         state_ != GridState::ArmTakeoff &&
         state_ != GridState::MarkerLockYaw);
    AltitudePolicyOutput alt_out = altitude_->update(alt_in);
    out.target_altitude_m = alt_out.target_altitude_m;
    out.altitude_off_pad_confirmed = alt_out.off_pad_confirmed;

    populateLineInputs(input, out);

    // Cycle 16: sliding-window marker stability. Push the most prominent
    // non-vertiport marker ID seen this frame (or -1 if none) into the
    // window. The window flushes itself when two distinct IDs appear inside
    // it, so transient mis-identifications never accumulate.
    {
        int observed = -1;
        if (input.vision) {
            for (const auto& m : input.vision->markers) {
                if (m.id == config_.vertiport_marker_id) continue;
                observed = m.id;
                break;
            }
        }
        marker_window_.push(observed);
    }

    // Dispatch
    switch (state_) {
    case GridState::Idle:
        out.intent = control::GridControlIntent::Idle;
        break;
    case GridState::ArmTakeoff:           handleArmTakeoff(input, out); break;
    case GridState::MarkerLockYaw:        handleMarkerLockYaw(input, out); break;
    case GridState::EntryForward:         handleEntryForward(input, out); break;
    case GridState::EntryCenterOrigin:    handleEntryCenterOrigin(input, out); break;
    case GridState::SnakeForward:         handleSnakeForward(input, out); break;
    case GridState::SnakeRecordNode:      handleSnakeRecordNode(input, out); break;
    case GridState::SnakeLaunchAlign:     handleSnakeLaunchAlign(input, out); break;
    case GridState::SnakeStopAtCenter:    handleSnakeStopAtCenter(input, out); break;
    case GridState::SnakeTurn90:          handleSnakeTurn90(input, out); break;
    case GridState::SnakeAdvanceOneCell:  handleSnakeAdvanceOneCell(input, out); break;
    case GridState::SnakeTurn90Again:     handleSnakeTurn90Again(input, out); break;
    case GridState::SnakeComplete:        handleSnakeComplete(input, out); break;
    case GridState::Land:                 handleLand(input, out); break;
    case GridState::EmergencyLand:        handleEmergencyLand(input, out); break;
    case GridState::Done:
        out.intent = control::GridControlIntent::Idle;
        out.mission_finished = true;
        break;
    }

    out.state = state_;
    if (tracker_) {
        out.current_coord = tracker_->currentCoord();
        out.current_heading = tracker_->currentHeading();
    }
    if (snake_) {
        out.snake_dir = snake_->currentSnakeDir();
    }
    out.intersections_recorded = intersections_recorded_;
    // Cycle 12 B: expose the fresh intersection center for the cy-feedback
    // deceleration in StopAndCenter intent.
    out.intersection_center_x_norm = intersectionCenterXNorm(input);
    out.intersection_center_y_norm = input.intersection_decision.center_y_norm;
    out.intersection_valid =
        input.vision && input.vision->intersection.valid &&
        input.intersection_decision.accepted_type != onboard::vision::IntersectionType::None;
    out.hop_distance_m = hopDistance(input);

    // Cycle 13: compute drone fractional position from last committed node
    // using LOCAL_NED displacement. The composition root forwards this to
    // telemetry so the GCS can render the heading arrow at a sub-cell position.
    // The displacement is projected onto the current grid heading because the
    // drone only moves forward in the grid frame (lateral motion is corrective
    // and bounded by the line controller).
    if (last_node_local_x_.has_value() && last_node_local_y_.has_value() &&
        input.local_x_m.has_value() && input.local_y_m.has_value() && tracker_) {
        const double dx = *input.local_x_m - *last_node_local_x_;
        const double dy = *input.local_y_m - *last_node_local_y_;
        const double dist = std::sqrt(dx * dx + dy * dy);
        const double cell = std::max(0.01, config_.cell_size_m);
        const double progress = std::clamp(dist / cell, 0.0, 2.0);

        double hx = 0.0;
        double hy = 0.0;
        switch (tracker_->currentHeading()) {
        case GridHeading::North: hy = -1.0; break;  // Cycle 13 revert: north = -y
        case GridHeading::South: hy = +1.0; break;
        case GridHeading::East:  hx = +1.0; break;
        case GridHeading::West:  hx = -1.0; break;
        case GridHeading::Unknown: break;
        }
        out.drone_position_valid = true;
        out.drone_cell_progress = progress;
        out.drone_grid_offset_x = progress * hx;
        out.drone_grid_offset_y = progress * hy;
    } else {
        out.drone_position_valid = false;
        out.drone_cell_progress = 0.0;
        out.drone_grid_offset_x = 0.0;
        out.drone_grid_offset_y = 0.0;
    }
    return out;
}

// Cycle 16: hop-to-hop helpers.
void GridMission::armHopStart(const GridMissionInput& in)
{
    if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
        hop_start_local_x_ = *in.local_x_m;
        hop_start_local_y_ = *in.local_y_m;
    } else {
        hop_start_local_x_.reset();
        hop_start_local_y_.reset();
    }
    hop_align_window_seen_ = false;
}

double GridMission::hopDistance(const GridMissionInput& in) const
{
    if (!hop_start_local_x_.has_value() || !hop_start_local_y_.has_value()) return 0.0;
    if (!in.local_x_m.has_value() || !in.local_y_m.has_value()) return 0.0;
    const double dx = *in.local_x_m - *hop_start_local_x_;
    const double dy = *in.local_y_m - *hop_start_local_y_;
    return std::sqrt(dx * dx + dy * dy);
}

bool GridMission::inHopAlignWindow(double distance) const
{
    return distance >= config_.hop_align_start_m &&
           distance <= config_.hop_align_end_m;
}

void GridMission::latchHopYawAfterAlign(
    const GridMissionInput& in,
    bool in_align_window,
    double distance,
    double& yaw_target_rad)
{
    if (in_align_window) {
        hop_align_window_seen_ = true;
        return;
    }
    if (hop_align_window_seen_ && distance > config_.hop_align_end_m) {
        if (in.attitude_yaw_rad.has_value()) {
            yaw_target_rad = wrap(*in.attitude_yaw_rad);
        }
        hop_align_window_seen_ = false;
    }
}

// --------------------------- Per-state handlers ---------------------------

void GridMission::handleArmTakeoff(const GridMissionInput& in, GridMissionOutput& out)
{
    out.request_arm_takeoff = true;
    out.takeoff_altitude_m = config_.vertiport_altitude_m;
    out.intent = control::GridControlIntent::Idle;  // autopilot.takeoff() handles climb
    const double trigger = config_.vertiport_altitude_m * 0.7;
    const bool rng_ok = in.rangefinder_m.has_value() && *in.rangefinder_m >= trigger;
    const bool alt_ok = in.local_altitude_m.has_value() && *in.local_altitude_m >= trigger;
    if (rng_ok || alt_ok) {
        // Cycle 16: latch the takeoff yaw and pre-compute the
        // right-90° target so MarkerLockYaw uses a fixed reference frame.
        if (in.attitude_yaw_rad.has_value()) {
            yaw_align_target_rad_ =
                wrap(*in.attitude_yaw_rad + config_.marker_lock_yaw_delta_rad);
        } else {
            yaw_align_target_rad_ = config_.marker_lock_yaw_delta_rad;
        }
        transition(GridState::MarkerLockYaw, in.now_s, "altitude_reached");
        vertiport_marker_stable_count_ = 0;
        vertiport_yaw_stable_count_ = 0;
    }
}

void GridMission::handleMarkerLockYaw(const GridMissionInput& in, GridMissionOutput& out)
{
    // Cycle 16: combined "marker centered + yaw +90°" pre-flight. Hovers on
    // top of the vertiport ArUco marker while rotating the body yaw by
    // marker_lock_yaw_delta_rad (default +π/2 = right 90°). Hands off to
    // EntryForward only when BOTH the marker is centered (|err| < tol) and
    // the yaw error is within tolerance for snake_yaw_stable_frames ticks.
    out.intent = control::GridControlIntent::MarkerHover;
    out.target_yaw_rad = yaw_align_target_rad_;
    out.target_altitude_m = config_.vertiport_altitude_m;

    auto marker = findMarker(in.vision, config_.vertiport_marker_id);
    if (marker.has_value()) {
        ++vertiport_marker_stable_count_;
        populateMarkerInputs(in, config_.vertiport_marker_id, out);
        if (vertiport_marker_stable_count_ >= config_.vertiport_marker_stable_frames) {
            if (registry_) {
                registry_->observe(marker->id, GridCoord{}, /*grid_coord_valid=*/false,
                                   marker->orientation_deg, in.timestamp_ms, in.frame_seq);
            }
            out.vertiport_verified = true;
        }
    } else {
        vertiport_marker_stable_count_ = 0;
        out.marker_detected = false;
    }

    bool yaw_ok = false;
    if (in.attitude_yaw_rad.has_value()) {
        const double err = std::abs(wrap(yaw_align_target_rad_ - *in.attitude_yaw_rad));
        if (err <= config_.vertiport_yaw_tolerance_rad) {
            ++vertiport_yaw_stable_count_;
        } else {
            vertiport_yaw_stable_count_ = 0;
        }
        yaw_ok = (vertiport_yaw_stable_count_ >= config_.vertiport_yaw_stable_frames);
    }

    const bool marker_centered =
        out.marker_detected &&
        std::abs(out.marker_center_error_x_norm) <= config_.marker_lock_center_tol_norm &&
        std::abs(out.marker_center_error_y_norm) <= config_.marker_lock_center_tol_norm;

    if (marker_centered && yaw_ok) {
        // Cycle 16: enter the free-flight forward phase with yaw frozen at
        // the post-rotation target. Latch hop_start so EntryForward can use
        // LOCAL_NED distance as a safety bound if needed.
        armHopStart(in);
        if (decision_engine_) {
            decision_engine_->reset();
        }
        entry_forward_start_frame_seq_ = in.frame_seq;
        transition(GridState::EntryForward, in.now_s, "marker_lock_done");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.vertiport_verify_timeout_s) {
        last_safety_event_ = "marker_lock_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "marker_lock_timeout");
    }
}

void GridMission::handleEntryForward(const GridMissionInput& in, GridMissionOutput& out)
{
    // Cycle 16: yaw-frozen forward scoot from the vertiport toward the grid.
    // There is no line between vertiport and the first grid intersection, so
    // this phase starts by intentionally ignoring vision: while still over
    // the vertiport texture, the ArUco pattern and pad artwork produce
    // convincing false L/T/+ candidates. Once LOCAL_NED says the vehicle has
    // cleared the pad, we wait for the real first grid intersection.
    out.intent = control::GridControlIntent::ForwardBlind;
    out.forward_speed_override_mps = config_.entry_forward_speed_mps;
    out.target_yaw_rad = yaw_align_target_rad_;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.vertiport_verified = true;

    const double progress = hopDistance(in);
    const double elapsed_s = in.now_s - state_entry_s_;
    const bool distance_clear = progress >= config_.entry_blind_clear_distance_m;
    const bool time_clear = elapsed_s >= config_.entry_blind_min_s;
    const std::uint32_t frames_since_entry =
        (entry_forward_start_frame_seq_ == 0 || in.frame_seq < entry_forward_start_frame_seq_)
        ? 0u
        : in.frame_seq - entry_forward_start_frame_seq_;
    const bool frames_clear =
        config_.entry_blind_min_frames <= 0 ||
        (frames_since_entry >= static_cast<std::uint32_t>(config_.entry_blind_min_frames));

    if (distance_clear && time_clear && frames_clear &&
        progress >= config_.entry_intersection_min_distance_m &&
        isEntryIntersectionCandidate(in)) {
        entry_center_stable_count_ = 0;
        if (decision_engine_) {
            decision_engine_->reset();
        }
        transition(GridState::EntryCenterOrigin, in.now_s, "first_intersection_seen");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.entry_forward_timeout_s) {
        last_safety_event_ = "entry_forward_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "entry_forward_timeout");
    }
}

void GridMission::handleEntryCenterOrigin(const GridMissionInput& in, GridMissionOutput& out)
{
    // Center the first grid intersection under the camera before declaring it
    // as local origin. This keeps the hop anchor aligned with the actual
    // (0,0) node instead of the first frame where the L/T/+ entered view.
    out.intent = control::GridControlIntent::IntersectionCenter;
    out.target_yaw_rad = yaw_align_target_rad_;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.vertiport_verified = true;

    const bool centered = isIntersectionCenteredForEntry(in);
    const bool velocity_ok =
        !in.local_velocity_xy_mps.has_value() ||
        *in.local_velocity_xy_mps <= config_.entry_center_velocity_threshold_mps;

    if (centered && velocity_ok) {
        ++entry_center_stable_count_;
    } else {
        entry_center_stable_count_ = 0;
    }

    if (entry_center_stable_count_ >= config_.entry_center_stable_frames) {
        latchGridOrigin(in, out);
        if (decision_engine_) {
            decision_engine_->reset();
        }
        snake_launch_align_stable_count_ = 0;
        transition(GridState::SnakeLaunchAlign, in.now_s, "origin_centered");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.entry_center_timeout_s) {
        last_safety_event_ = "entry_center_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "entry_center_timeout");
    }
}

void GridMission::handleSnakeForward(const GridMissionInput& in, GridMissionOutput& out)
{
    // Cycle 16: hop-to-hop. From the last committed node, fly yaw-frozen
    // body-forward for the whole cell. At the configured align window
    // (default 1.4-1.7m from the hop start) we open a single LineFollow
    // burst so vision can correct lateral drift + yaw bias once per cell.
    // Outside that window the controller stays in ForwardBlind, which keeps
    // yaw fully locked and ignores any LineDetector arrows.
    //
    // The next node arrival is detected by intersection.valid AND a minimum
    // hop distance gate so the just-departed node's residual lookahead does
    // not retrigger.

    if (!hop_start_local_x_.has_value()) {
        armHopStart(in);
    }
    out.target_altitude_m = config_.cruise_altitude_m;

    const double distance = hopDistance(in);

    // Cycle 12 C: blind forward immediately after SnakeTurn90Again so we
    // physically enter the new column before any vision-driven correction
    // can fight the rotation.
    const bool post_turn_blind = in.now_s < snake_post_turn_blind_until_s_;
    const bool in_align_window = !post_turn_blind && inHopAlignWindow(distance);
    latchHopYawAfterAlign(in, in_align_window, distance, yaw_align_target_rad_);
    out.target_yaw_rad = yaw_align_target_rad_;

    if (in_align_window) {
        out.intent = control::GridControlIntent::LineFollow;
    } else {
        out.intent = control::GridControlIntent::ForwardBlind;
    }

    // Line lost tracking — only matters during the LineFollow window. In
    // ForwardBlind we do not require a line.
    if (in_align_window) {
        if (in.vision && in.vision->line.detected) {
            line_lost_start_s_ = -1.0;
        } else if (line_lost_start_s_ < 0.0) {
            line_lost_start_s_ = in.now_s;
        }
    } else {
        line_lost_start_s_ = -1.0;
    }

    const bool lockout_active =
        (last_node_record_s_ > 0.0) &&
        (in.now_s - last_node_record_s_ < config_.snake_record_lockout_s);
    const bool turn_lockout_active =
        (last_turn_complete_s_ > 0.0) &&
        (in.now_s - last_turn_complete_s_ < config_.snake_turn_lockout_s);

    // Boundary watchdog: once idec independently decides a turn is required
    // (state == TurnReady) and we are past the post-record grace period,
    // promote to SnakeStopAtCenter even if NodeRecord events are silent.
    const double since_last_record_s = (last_node_record_s_ > 0.0)
        ? (in.now_s - last_node_record_s_)
        : config_.snake_post_record_grace_s + 1.0;
    // Cycle 16: also require a minimum hop distance before the watchdog can
    // fire. Without this, the vertiport texture's residual cross artwork can
    // trip TurnReady the moment we enter SnakeForward and immediately
    // promote to SnakeStopAtCenter -> SnakeComplete -> LAND.
    if (!post_turn_blind &&
        since_last_record_s >= config_.snake_post_record_grace_s &&
        distance >= config_.hop_intersection_min_distance_m &&
        in.intersection_decision.required_turn &&
        in.intersection_decision.state == IntersectionDecisionState::TurnReady) {
        last_node_front_open_ = forwardBranchPresent(in.intersection_decision);
        last_node_grid_branch_mask_ = rotateCameraBranchMaskToGrid(
            in.intersection_decision.accepted_branch_mask,
            tracker_ ? tracker_->currentHeading() : GridHeading::North);
        out.last_safety_event = "";
        transition(GridState::SnakeStopAtCenter, in.now_s, "boundary_watchdog");
        snake_stop_stable_count_ = 0;
        return;
    }

    // Cycle 16: next-node arrival. The hop distance gate replaces the old
    // 0.5×cell_size LOCAL_NED gate from Cycle 8.
    const bool arrival_distance_ok = distance >= config_.hop_intersection_min_distance_m;

    if (!post_turn_blind && !lockout_active && !turn_lockout_active && arrival_distance_ok &&
        nodeJustRecorded(in.intersection_decision) && in.node_event.valid) {
        ++intersections_recorded_;
        out.commit_tracker_advance = true;
        last_node_grid_branch_mask_ = in.node_event.grid_branch_mask;
        last_node_front_open_ = forwardBranchPresent(in.intersection_decision);
        if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
            last_node_local_x_ = *in.local_x_m;
            last_node_local_y_ = *in.local_y_m;
        }

        if (registry_ && registry_->gridMarkerCount() >= static_cast<std::size_t>(config_.markers_expected)) {
            transition(GridState::SnakeRecordNode, in.now_s, "record_pre_complete");
            last_node_record_s_ = in.now_s;
            return;
        }
        if (!forwardBranchPresent(in.intersection_decision) ||
            in.node_event.camera_branch_mask == 0) {
            transition(GridState::SnakeRecordNode, in.now_s, "record_boundary");
            last_node_record_s_ = in.now_s;
            return;
        }
        transition(GridState::SnakeRecordNode, in.now_s, "record_node");
        last_node_record_s_ = in.now_s;
        return;
    }

    // Distance failsafe — if we have gone well past one cell length without
    // seeing the next intersection, hand off to the EmergencyLand path.
    if (distance > config_.hop_max_distance_m) {
        last_safety_event_ = "hop_distance_exceeded";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "hop_distance_exceeded");
    }
}

void GridMission::handleSnakeRecordNode(const GridMissionInput& in, GridMissionOutput& out)
{
    // Default: line-follow with deceleration through the node (Cycle 9 Option C).
    // Cycle 16: hold position during the short dwell so vision has clean
    // frames to settle the branch classification. Marker nodes override to
    // MarkerHover further down.
    out.intent = control::GridControlIntent::HoldPosition;
    out.target_yaw_rad = yaw_align_target_rad_;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.advance_phase = false;

    // Cycle 16: pick the marker focus from the sliding window. If a stable
    // ID was latched on a previous tick we keep it for the rest of the dwell
    // even when the camera momentarily loses the marker.
    int focus_marker_id = -1;
    if (marker_hover_active_ && last_recorded_marker_id_ >= 0) {
        focus_marker_id = last_recorded_marker_id_;
    } else {
        const int stable_id = marker_window_.bestStableId();
        if (stable_id >= 0 &&
            stable_id != config_.vertiport_marker_id &&
            (!registry_ || !registry_->hasId(stable_id))) {
            focus_marker_id = stable_id;
        }
    }
    if (focus_marker_id >= 0) {
        marker_hover_active_ = true;
        last_recorded_marker_id_ = focus_marker_id;
        populateMarkerInputs(in, focus_marker_id, out);
        out.intent = control::GridControlIntent::MarkerHover;
        out.advance_phase = false;  // full hover during marker hold

        if (registry_) {
            auto observation = findMarker(in.vision, focus_marker_id);
            const bool was_new = !registry_->hasId(focus_marker_id);
            registry_->observe(focus_marker_id,
                               tracker_ ? tracker_->currentCoord() : GridCoord{},
                               /*grid_coord_valid=*/true,
                               observation.has_value() ? observation->orientation_deg : 0.0f,
                               in.timestamp_ms, in.frame_seq);
            if (was_new) {
                const GridCoord c = tracker_ ? tracker_->currentCoord() : GridCoord{};
                std::cerr << "[marker-commit] id=" << focus_marker_id
                          << " coord=(" << c.x << "," << c.y << ")"
                          << " t=" << in.now_s << "\n";
            }
        }
    }

    const double dwell_s = marker_hover_active_
        ? config_.snake_marker_hover_s
        : config_.snake_record_dwell_s;
    if (in.now_s - state_entry_s_ >= dwell_s) {
        marker_hover_active_ = false;
        // Cycle 10: re-evaluate branches with the FRESH idec at dwell end.
        // The branch topology can resolve during the dwell (e.g. T -> L as the
        // drone closes in on a corner), and we must not stick with the
        // commit-time cached classification.
        const bool fresh_front_open =
            forwardBranchPresent(in.intersection_decision);
        const std::uint8_t fresh_grid_mask = rotateCameraBranchMaskToGrid(
            in.intersection_decision.accepted_branch_mask,
            tracker_ ? tracker_->currentHeading() : GridHeading::North);
        last_node_front_open_ = fresh_front_open;
        last_node_grid_branch_mask_ = fresh_grid_mask;

        const bool all_markers_found = registry_ &&
            registry_->gridMarkerCount() >= static_cast<std::size_t>(config_.markers_expected);
        if (all_markers_found) {
            transition(GridState::SnakeComplete, in.now_s, "all_markers");
            return;
        }
        if (!fresh_front_open || fresh_grid_mask == 0) {
            transition(GridState::SnakeStopAtCenter, in.now_s, "boundary_detected");
            snake_stop_stable_count_ = 0;
            return;
        }
        snake_launch_align_stable_count_ = 0;
        transition(GridState::SnakeLaunchAlign, in.now_s, "node_done");
    }
}

void GridMission::handleSnakeLaunchAlign(const GridMissionInput& in, GridMissionOutput& out)
{
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yaw_align_target_rad_;

    const bool centered = isIntersectionCenteredForEntry(in);
    const bool velocity_ok =
        !in.local_velocity_xy_mps.has_value() ||
        *in.local_velocity_xy_mps <= config_.entry_center_velocity_threshold_mps;
    const bool timeout =
        in.now_s - state_entry_s_ > config_.snake_launch_align_timeout_s;

    if (!centered || !velocity_ok) {
        out.intent = control::GridControlIntent::IntersectionCenter;
        snake_launch_align_stable_count_ = 0;
        if (!timeout) {
            return;
        }
    }

    const bool front_open = forwardBranchPresent(in.intersection_decision) || last_node_front_open_;
    const bool line_ready =
        front_open &&
        out.line_detected &&
        out.line_confidence >= config_.snake_launch_line_min_confidence;

    if (line_ready) {
        out.intent = control::GridControlIntent::LaunchAlign;
        const bool line_centered =
            std::abs(out.line_center_error_norm) <=
                config_.snake_launch_line_center_tolerance_norm;
        const bool yaw_aligned =
            std::abs(out.line_angle_error_rad) <=
                config_.snake_launch_line_angle_tolerance_rad;
        if (line_centered && yaw_aligned && velocity_ok) {
            ++snake_launch_align_stable_count_;
        } else {
            snake_launch_align_stable_count_ = 0;
        }
        if (snake_launch_align_stable_count_ <
            config_.snake_launch_align_stable_frames) {
            if (!timeout) {
                return;
            }
        }
    } else {
        out.intent = control::GridControlIntent::HoldPosition;
        snake_launch_align_stable_count_ = 0;
        if (!timeout) {
            return;
        }
    }

    // Cycle 17: removed `yaw_align_target_rad_ = wrap(*attitude_yaw_rad)`
    // here. Re-latching attitude into yaw_align_target_rad_ at LaunchAlign
    // exit froze any per-cell LaunchAlign yaw jitter (driven by the
    // line_controller's angle_yaw_kp) into the next hop's reference,
    // causing the cumulative drift observed over 7-8 cells. The latched
    // reference must only change at MarkerLockYaw / ArmTakeoff (initial)
    // and at SnakeTurn90Again completion (column transition).
    armHopStart(in);
    snake_launch_align_stable_count_ = 0;
    transition(GridState::SnakeForward, in.now_s,
               timeout ? "launch_align_timeout" : "launch_aligned");
}

void GridMission::handleSnakeStopAtCenter(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::StopAndCenter;
    const double vmag = in.local_velocity_xy_mps.value_or(1.0);
    if (vmag <= config_.snake_stop_velocity_threshold_mps) {
        ++snake_stop_stable_count_;
    } else {
        snake_stop_stable_count_ = 0;
    }

    // Cycle 21: while we decelerate AND during the post-settle window, keep
    // overwriting last_node_grid_branch_mask_ with the freshest "node-like"
    // idec frame. The watchdog-fire frame can be a transient (e.g. a 1-frame
    // 0x07 in camera that excludes the real boundary's side branch). A T
    // intersection has 3 branches; a + has 4; both at least 3-bit popcount.
    // Two-bit (straight) frames are ignored so a cruise transient does not
    // dilute the boundary mask back to a non-boundary state.
    {
        const auto idec_state = in.intersection_decision.state;
        if (idec_state == IntersectionDecisionState::TurnReady ||
            idec_state == IntersectionDecisionState::TurnConfirm ||
            idec_state == IntersectionDecisionState::NodeRecord) {
            const std::uint8_t fresh = rotateCameraBranchMaskToGrid(
                in.intersection_decision.accepted_branch_mask,
                tracker_ ? tracker_->currentHeading() : GridHeading::North);
            int bits = 0;
            for (std::uint8_t m = fresh; m; m &= static_cast<std::uint8_t>(m - 1)) ++bits;
            if (bits >= 3) {
                last_node_grid_branch_mask_ = fresh;
                last_node_front_open_ = forwardBranchPresent(in.intersection_decision);
            }
        }
    }

    if (snake_stop_stable_count_ < config_.snake_stop_velocity_consecutive_frames) {
        return;
    }
    // Velocity settled — wait an extra settle window so vision can finalise
    // the branch mask before we commit to a direction.
    if (snake_stop_settle_entry_s_ < 0.0) {
        snake_stop_settle_entry_s_ = in.now_s;
    }
    if (in.now_s - snake_stop_settle_entry_s_ < config_.snake_boundary_settle_s) {
        return;
    }
    snake_stop_settle_entry_s_ = -1.0;
    {
        // Ask SnakePlanner for direction at this boundary.
        // Cycle 11: in.node_event.grid_branch_mask is 0 in subsequent ticks
        // because peek returns valid=false once idec lockout is active. Use
        // the cached mask latched by the boundary watchdog (or SnakeRecordNode
        // dwell-end) which reflects the actual branches at the node.
        SnakePlannerInput sp_in;
        sp_in.heading = tracker_ ? tracker_->currentHeading() : GridHeading::Unknown;
        sp_in.grid_branch_mask = last_node_grid_branch_mask_;
        sp_in.at_boundary_decision = true;
        sp_in.consecutive_boundary_failure = consecutive_boundary_failures_ >= 2;
        auto sp_out = snake_ ? snake_->planAtBoundary(sp_in) : SnakePlannerOutput{};
        if (sp_out.action == SnakeAction::Complete) {
            ++consecutive_boundary_failures_;
            transition(GridState::SnakeComplete, in.now_s, "boundary_complete");
            return;
        }
        pending_turn_dir_ = sp_out.plan_turn_dir;
        pending_post_turn_heading_ = sp_out.next_heading;
        // Cycle 17: compute the first-turn target from the latched
        // yaw_align_target_rad_ (column reference) rather than the live
        // attitude_yaw_rad. Otherwise any per-cell drift propagates into
        // the post-turn heading and accumulates across columns.
        {
            const double dir = (pending_turn_dir_ == SnakeTurnDir::Right) ? +1.0 : -1.0;
            yaw_target_rad_ = wrap(yaw_align_target_rad_ + dir * (M_PI / 2.0));
        }
        if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
            turn_origin_x_ = *in.local_x_m;
            turn_origin_y_ = *in.local_y_m;
        }
        snake_yaw_stable_count_ = 0;
        transition(GridState::SnakeTurn90, in.now_s, "stopped");
    }
}

void GridMission::handleSnakeTurn90(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::YawTurn;
    out.target_yaw_rad = yaw_target_rad_;
    if (in.attitude_yaw_rad.has_value()) {
        const double err = std::abs(wrap(yaw_target_rad_ - *in.attitude_yaw_rad));
        if (err <= config_.snake_yaw_target_tolerance_rad) {
            ++snake_yaw_stable_count_;
        } else {
            snake_yaw_stable_count_ = 0;
        }
        if (snake_yaw_stable_count_ >= config_.snake_yaw_stable_frames) {
            if (tracker_ && pending_post_turn_heading_ != GridHeading::Unknown) {
                tracker_->notifyTurnCompleted(pending_post_turn_heading_);
            }
            if (snake_) snake_->notifyFirstTurnCompleted();
            last_turn_complete_s_ = in.now_s;
            snake_turn_first_done_ = true;
            // Cycle 12 C: arm blind forward so the next state starts by driving
            // into the new corridor before re-engaging line tracking.
            snake_post_turn_blind_until_s_ =
                in.now_s + config_.snake_post_turn_blind_s;
            // Cycle 16: arm a fresh hop anchor — the column-transition cell
            // starts here, not at the boundary node.
            armHopStart(in);
            transition(GridState::SnakeAdvanceOneCell, in.now_s, "turn1_done");
        }
    }
}

void GridMission::handleSnakeAdvanceOneCell(const GridMissionInput& in, GridMissionOutput& out)
{
    // Cycle 12 C: blind forward immediately after the turn so the camera
    // clears the old corridor's line before we try to track again.
    if (!hop_start_local_x_.has_value()) {
        armHopStart(in);
    }
    out.target_altitude_m = config_.cruise_altitude_m;

    const double distance = hopDistance(in);

    const bool post_turn_blind = in.now_s < snake_post_turn_blind_until_s_;
    const bool in_align_window = !post_turn_blind && inHopAlignWindow(distance);
    latchHopYawAfterAlign(in, in_align_window, distance, yaw_target_rad_);
    out.target_yaw_rad = yaw_target_rad_;
    if (in_align_window) {
        out.intent = control::GridControlIntent::LineFollow;
    } else {
        out.intent = control::GridControlIntent::ForwardBlind;
    }

    if (post_turn_blind) {
        // Hard timeout still applies (snake_advance_timeout_s).
        if (in.now_s - state_entry_s_ > config_.snake_advance_timeout_s) {
            last_safety_event_ = "advance_timeout";
            out.last_safety_event = last_safety_event_;
            transition(GridState::EmergencyLand, in.now_s, "advance_timeout");
        }
        return;
    }

    const bool turn_lockout = in.now_s - last_turn_complete_s_ < config_.snake_turn_lockout_s;
    const bool arrival_distance_ok = distance >= config_.hop_intersection_min_distance_m;

    if (!turn_lockout && arrival_distance_ok &&
        nodeJustRecorded(in.intersection_decision) && in.node_event.valid) {
        ++intersections_recorded_;
        last_node_record_s_ = in.now_s;

        // Cycle 15: commit the column-transition cell to the tracker so the
        // subsequent SnakeForward starts from the new column's first row, not
        // from the old column's last row.
        out.commit_tracker_advance = true;
        if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
            last_node_local_x_ = *in.local_x_m;
            last_node_local_y_ = *in.local_y_m;
        }

        // Cycle 17: second-turn target also derives from yaw_target_rad_
        // (which itself is yaw_align_target_rad_ + first dir·π/2), so the
        // new column's reference is exactly the original ± π — drift-free.
        {
            const double dir = (pending_turn_dir_ == SnakeTurnDir::Right) ? +1.0 : -1.0;
            yaw_target_rad_ = wrap(yaw_target_rad_ + dir * (M_PI / 2.0));
        }
        snake_yaw_stable_count_ = 0;
        transition(GridState::SnakeTurn90Again, in.now_s, "cell_reached");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.snake_advance_timeout_s ||
        distance > config_.hop_max_distance_m) {
        last_safety_event_ = "advance_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "advance_timeout");
    }
}

void GridMission::handleSnakeTurn90Again(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::YawTurn;
    out.target_yaw_rad = yaw_target_rad_;
    if (in.attitude_yaw_rad.has_value()) {
        const double err = std::abs(wrap(yaw_target_rad_ - *in.attitude_yaw_rad));
        if (err <= config_.snake_yaw_target_tolerance_rad) {
            ++snake_yaw_stable_count_;
        } else {
            snake_yaw_stable_count_ = 0;
        }
        if (snake_yaw_stable_count_ >= config_.snake_yaw_stable_frames) {
            // Apply same direction twice; update tracker heading accordingly.
            if (tracker_) {
                GridHeading h = tracker_->currentHeading();
                GridHeading next = pending_turn_dir_ == SnakeTurnDir::Right ? turnRight(h) : turnLeft(h);
                tracker_->notifyTurnCompleted(next);
            }
            if (snake_) snake_->notifySecondTurnCompleted();
            last_turn_complete_s_ = in.now_s;
            snake_turn_first_done_ = false;
            consecutive_boundary_failures_ = 0;
            // Cycle 12 C: blind forward into the new column before SnakeForward
            // re-engages line tracking.
            snake_post_turn_blind_until_s_ =
                in.now_s + config_.snake_post_turn_blind_s;
            yaw_align_target_rad_ = yaw_target_rad_;
            snake_launch_align_stable_count_ = 0;
            transition(GridState::SnakeLaunchAlign, in.now_s, "turn2_done");
        }
    }
}

void GridMission::handleSnakeComplete(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::HoldPosition;
    if (in.now_s - state_entry_s_ > config_.snake_complete_hover_s) {
        transition(GridState::Land, in.now_s, "snake_done");
    }
}

void GridMission::handleLand(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::Land;
    out.request_land_mode = true;
    if (!in.armed) {
        out.mission_finished = true;
        transition(GridState::Done, in.now_s, "disarmed");
    }
}

void GridMission::handleEmergencyLand(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::Land;
    out.request_land_mode = true;
    out.mission_aborted = true;
    if (!in.armed) {
        transition(GridState::Done, in.now_s, "disarmed");
    }
}

} // namespace onboard::mission
