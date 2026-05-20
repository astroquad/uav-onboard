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
    case GridState::VertiportVerify:      return "VERTIPORT_VERIFY";
    case GridState::VertiportYawAlign:    return "VERTIPORT_YAW_ALIGN";
    case GridState::OffPadForward:        return "OFF_PAD_FORWARD";
    case GridState::LineEnter:            return "LINE_ENTER";
    case GridState::GridOriginLock:       return "GRID_ORIGIN_LOCK";
    case GridState::SnakeForward:         return "SNAKE_FORWARD";
    case GridState::SnakeRecordNode:      return "SNAKE_RECORD_NODE";
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
    snake_stop_stable_count_ = 0;
    snake_yaw_stable_count_ = 0;
    off_pad_line_stable_count_ = 0;
    origin_latched_ = false;
    line_lost_since_node_count_ = 0;
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
    line_enter_last_seen_s_ = -1.0;
    in_line_enter_relax_ = false;
    last_safety_event_.clear();
    last_node_grid_branch_mask_ = 0;
    last_node_front_open_ = false;
    marker_candidate_count_.clear();
    origin_published_ = false;
    snake_post_turn_blind_until_s_ = -1.0;
    if (decision_engine_) {
        decision_engine_->setNodeRecordYBand(node_record_y_min_default_,
                                             node_record_y_max_default_);
    }
}

std::size_t GridMission::stableMarkerCandidateCount() const
{
    std::size_t count = 0;
    for (const auto& [id, frames] : marker_candidate_count_) {
        if (frames >= config_.marker_observation_min_frames) ++count;
    }
    return count;
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

    // Cycle 16: explicit type gate. Anything other than Straight — including
    // Unknown — produces an immediate yaw + center freeze and short-circuits
    // the rest of the line-follow inputs. Approach to a real intersection
    // produces a brief Straight -> Unknown -> T/L/+ transition where the
    // LineDetector's fused-contour fit yields a misleading arrow; honouring
    // it would yaw the drone toward the cross branch.
    const auto t_acc = in.intersection_decision.accepted_type;
    if (t_acc != onboard::vision::IntersectionType::Straight) {
        out.line_angle_error_rad = 0.0;
        out.line_center_error_norm = 0.0;
        return;
    }

    // Cycle 15: even when accepted_type == Straight, require the Front + Back
    // branch arrows (the colored arrows the GCS overlay shows at the line
    // endpoints) to be ~180° apart within ±10°. This rejects the borderline
    // case where the classification is still Straight but a T is entering
    // the camera frame and tilting the per-branch fit.
    //
    // Note: `branches[]` lives on the raw VisionResult.intersection, not on
    // IntersectionDecision. accepted_type drives the topology gate (above);
    // the raw branch angles drive this head/tail straightness gate.
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
    if (input.local_x_m.has_value() && input.local_y_m.has_value()) {
        const double dx = *input.local_x_m - off_pad_origin_x_;
        const double dy = *input.local_y_m - off_pad_origin_y_;
        alt_in.distance_from_pad_m = std::sqrt(dx * dx + dy * dy);
    }
    alt_in.off_pad_requested =
        (state_ != GridState::Idle &&
         state_ != GridState::ArmTakeoff &&
         state_ != GridState::VertiportVerify &&
         state_ != GridState::VertiportYawAlign);
    AltitudePolicyOutput alt_out = altitude_->update(alt_in);
    out.target_altitude_m = alt_out.target_altitude_m;
    out.altitude_off_pad_confirmed = alt_out.off_pad_confirmed;

    populateLineInputs(input, out);

    // Cycle 9: marker stability — increment count for every ID visible this
    // frame, reset for IDs that vanished. Only markers with count >=
    // marker_observation_min_frames are treated as real downstream.
    {
        std::unordered_map<int, bool> seen_this_frame;
        if (input.vision) {
            for (const auto& m : input.vision->markers) {
                if (m.id == config_.vertiport_marker_id) continue;
                seen_this_frame[m.id] = true;
                auto it = marker_candidate_count_.find(m.id);
                if (it == marker_candidate_count_.end()) {
                    marker_candidate_count_[m.id] = 1;
                } else {
                    it->second = std::min(it->second + 1,
                        std::max(1, config_.marker_observation_min_frames * 4));
                }
            }
        }
        // Reset counters for IDs not seen this frame.
        for (auto it = marker_candidate_count_.begin();
             it != marker_candidate_count_.end(); ) {
            if (seen_this_frame.find(it->first) == seen_this_frame.end()) {
                it = marker_candidate_count_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // Dispatch
    switch (state_) {
    case GridState::Idle:
        out.intent = control::GridControlIntent::Idle;
        break;
    case GridState::ArmTakeoff:           handleArmTakeoff(input, out); break;
    case GridState::VertiportVerify:      handleVertiportVerify(input, out); break;
    case GridState::VertiportYawAlign:    handleVertiportYawAlign(input, out); break;
    case GridState::OffPadForward:        handleOffPadForward(input, out); break;
    case GridState::LineEnter:            handleLineEnter(input, out); break;
    case GridState::GridOriginLock:       handleGridOriginLock(input, out); break;
    case GridState::SnakeForward:         handleSnakeForward(input, out); break;
    case GridState::SnakeRecordNode:      handleSnakeRecordNode(input, out); break;
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
    out.intersection_center_y_norm = input.intersection_decision.center_y_norm;
    out.intersection_valid =
        input.intersection_decision.accepted_type != onboard::vision::IntersectionType::None;

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

// --------------------------- Per-state handlers ---------------------------

void GridMission::handleArmTakeoff(const GridMissionInput& in, GridMissionOutput& out)
{
    out.request_arm_takeoff = true;
    out.takeoff_altitude_m = config_.vertiport_altitude_m;
    out.intent = control::GridControlIntent::Idle;  // autopilot.takeoff() handles climb
    const double trigger = config_.vertiport_altitude_m * 0.7;  // 0.91m at 1.3m target
    const bool rng_ok = in.rangefinder_m.has_value() && *in.rangefinder_m >= trigger;
    const bool alt_ok = in.local_altitude_m.has_value() && *in.local_altitude_m >= trigger;
    if (rng_ok || alt_ok) {
        transition(GridState::VertiportVerify, in.now_s, "altitude_reached");
        vertiport_marker_stable_count_ = 0;
    }
}

void GridMission::handleVertiportVerify(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::HoldPosition;
    auto marker = findMarker(in.vision, config_.vertiport_marker_id);
    if (marker.has_value()) {
        ++vertiport_marker_stable_count_;
        if (vertiport_marker_stable_count_ >= config_.vertiport_marker_stable_frames) {
            // Mark vertiport as seen but NOT bound to grid coord yet.
            if (registry_) {
                registry_->observe(marker->id, GridCoord{}, /*grid_coord_valid=*/false,
                                   marker->orientation_deg, in.timestamp_ms, in.frame_seq);
            }
            // Latch yaw alignment target from marker orientation (rad).
            const double marker_yaw_rad = marker->orientation_deg * M_PI / 180.0;
            // Vehicle should align to vertiport orientation; we just lock current yaw + marker_yaw delta.
            if (in.attitude_yaw_rad.has_value()) {
                yaw_align_target_rad_ = wrap(*in.attitude_yaw_rad - marker_yaw_rad);
            } else {
                yaw_align_target_rad_ = -marker_yaw_rad;
            }
            vertiport_yaw_stable_count_ = 0;
            transition(GridState::VertiportYawAlign, in.now_s, "vertiport_locked");
        }
    } else {
        vertiport_marker_stable_count_ = 0;
        if (in.now_s - state_entry_s_ > config_.vertiport_verify_timeout_s) {
            out.last_safety_event = "vertiport_verify_timeout";
            transition(GridState::EmergencyLand, in.now_s, "vertiport_timeout");
        }
    }
    out.vertiport_verified = vertiport_marker_stable_count_ >= config_.vertiport_marker_stable_frames;
}

void GridMission::handleVertiportYawAlign(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::YawAlign;
    out.target_yaw_rad = yaw_align_target_rad_;
    out.vertiport_verified = true;
    if (in.attitude_yaw_rad.has_value()) {
        const double err = std::abs(wrap(yaw_align_target_rad_ - *in.attitude_yaw_rad));
        if (err <= config_.vertiport_yaw_tolerance_rad) {
            ++vertiport_yaw_stable_count_;
        } else {
            vertiport_yaw_stable_count_ = 0;
        }
        if (vertiport_yaw_stable_count_ >= config_.vertiport_yaw_stable_frames) {
            if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
                off_pad_origin_x_ = *in.local_x_m;
                off_pad_origin_y_ = *in.local_y_m;
            }
            transition(GridState::OffPadForward, in.now_s, "yaw_aligned");
            off_pad_line_stable_count_ = 0;
        }
    } else {
        // Without yaw, skip alignment and proceed (fail-soft).
        if (in.now_s - state_entry_s_ > 2.0) {
            if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
                off_pad_origin_x_ = *in.local_x_m;
                off_pad_origin_y_ = *in.local_y_m;
            }
            transition(GridState::OffPadForward, in.now_s, "yaw_unknown_skip");
        }
    }
}

void GridMission::handleOffPadForward(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::ForwardBlind;
    out.vertiport_verified = true;

    double distance_m = 0.0;
    if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
        const double dx = *in.local_x_m - off_pad_origin_x_;
        const double dy = *in.local_y_m - off_pad_origin_y_;
        distance_m = std::sqrt(dx * dx + dy * dy);
    }

    // Line trigger disabled here: while still over the vertiport texture, the
    // 'V' graphic can read as a line. Use only LOCAL_NED distance to clear the
    // pad before handing off to LineEnter.
    const bool distance_trigger = distance_m >= config_.off_pad_distance_trigger_m;
    if (distance_trigger) {
        transition(GridState::LineEnter, in.now_s, "distance_reached");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.off_pad_timeout_s) {
        out.last_safety_event = "off_pad_timeout";
        transition(GridState::EmergencyLand, in.now_s, "off_pad_timeout");
    }
}

void GridMission::handleLineEnter(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::LineFollow;

    // Cycle 8: relax NodeRecord Y band on entry so the first vertiport->grid
    // intersection latches even when it appears slightly above center.
    if (decision_engine_ && !in_line_enter_relax_) {
        decision_engine_->setNodeRecordYBand(node_record_y_min_line_enter_,
                                             node_record_y_max_default_);
        in_line_enter_relax_ = true;
    }

    if (nodeJustRecorded(in.intersection_decision) && in.node_event.valid) {
        transition(GridState::GridOriginLock, in.now_s, "first_node");
        return;
    }

    // Cycle 8: reset timeout counter while a line is still detected. Only
    // pure idle (no line) for the full timeout window triggers EmergencyLand.
    if (in.vision && in.vision->line.detected) {
        line_enter_last_seen_s_ = in.now_s;
    }
    const double idle_s = (line_enter_last_seen_s_ > 0.0)
        ? (in.now_s - line_enter_last_seen_s_)
        : (in.now_s - state_entry_s_);
    if (idle_s > config_.line_enter_timeout_s) {
        out.last_safety_event = "line_enter_timeout";
        transition(GridState::EmergencyLand, in.now_s, "line_enter_timeout");
    }
}

void GridMission::handleGridOriginLock(const GridMissionInput& in, GridMissionOutput& out)
{
    if (tracker_ && !origin_latched_) {
        tracker_->forceOrigin(GridCoord{0, 0}, GridHeading::North);
        origin_latched_ = true;
    }
    intersections_recorded_ = 1; // first node is (0,0)
    last_node_record_s_ = in.now_s;
    // Cycle 8: restore the strict NodeRecord band now that we are inside the
    // grid, and latch the (0,0) position for the distance gate.
    if (decision_engine_) {
        decision_engine_->setNodeRecordYBand(node_record_y_min_default_,
                                             node_record_y_max_default_);
    }
    in_line_enter_relax_ = false;
    if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
        last_node_local_x_ = *in.local_x_m;
        last_node_local_y_ = *in.local_y_m;
    }
    // Cycle 12 A: emit a synthetic GridNodeEvent for the origin (0,0) so the
    // GCS GridMapTracker observes it as the first committed node. With
    // first_node=true and arrival_heading=north, GCS computes startCoord =
    // (0,0) - heading_vector(north) = (0,-1) (using the Cycle 12 flipped
    // headingVector), which puts `s` at the vertiport position.
    if (!origin_published_) {
        GridNodeEvent origin_event;
        origin_event.valid = true;
        origin_event.node_id = 1;
        origin_event.local_coord = GridCoord{0, 0};
        origin_event.topology = onboard::vision::IntersectionType::Unknown;
        origin_event.arrival_heading = GridHeading::North;
        origin_event.camera_branch_mask = 0;
        origin_event.grid_branch_mask = 0;
        origin_event.first_node = true;
        origin_event.origin_local_only = true;
        out.origin_publish_event = origin_event;
        origin_published_ = true;
    }
    out.intent = control::GridControlIntent::LineFollow;
    transition(GridState::SnakeForward, in.now_s, "origin_locked");
}

void GridMission::handleSnakeForward(const GridMissionInput& in, GridMissionOutput& out)
{
    // Cycle 12 C: blind forward after SnakeTurn90Again so we drive into the
    // new column before re-engaging line tracking. The boundary watchdog and
    // node-record logic below are bypassed for this short window.
    if (in.now_s < snake_post_turn_blind_until_s_) {
        out.intent = control::GridControlIntent::ForwardBlind;
        // Reset line-lost timer and distance-gate anchor for a clean start.
        line_lost_start_s_ = -1.0;
        if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
            last_node_local_x_ = *in.local_x_m;
            last_node_local_y_ = *in.local_y_m;
        }
        return;
    }

    out.intent = control::GridControlIntent::LineFollow;

    // Line lost tracking
    if (in.vision && in.vision->line.detected) {
        line_lost_start_s_ = -1.0;
    } else if (line_lost_start_s_ < 0.0) {
        line_lost_start_s_ = in.now_s;
    }
    if (line_lost_start_s_ > 0.0 &&
        in.now_s - line_lost_start_s_ > config_.snake_line_lost_emergency_s) {
        out.last_safety_event = "line_lost_emergency";
        transition(GridState::EmergencyLand, in.now_s, "line_lost");
        return;
    }

    const bool lockout_active =
        (last_node_record_s_ > 0.0) &&
        (in.now_s - last_node_record_s_ < config_.snake_record_lockout_s);
    const bool turn_lockout_active =
        (last_turn_complete_s_ > 0.0) &&
        (in.now_s - last_turn_complete_s_ < config_.snake_turn_lockout_s);

    // Cycle 10 boundary watchdog: after the post-record grace window, if idec
    // independently concludes "turn required" (state == TurnConfirm/TurnReady),
    // promote to SnakeStopAtCenter immediately. NodeRecord events stop firing
    // once a boundary is reached because idec is locked into the turn states,
    // so without this we would just sail off the line.
    const double since_last_record_s = (last_node_record_s_ > 0.0)
        ? (in.now_s - last_node_record_s_)
        : config_.snake_post_record_grace_s + 1.0;
    // Cycle 11: only TurnReady — idec confirms the intersection is in the
    // turn_zone Y band, i.e. the drone is physically over the node. TurnConfirm
    // fires earlier while still approaching and would stop the drone mid-cell.
    if (since_last_record_s >= config_.snake_post_record_grace_s &&
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

    // Cycle 8 distance gate: require LOCAL_NED displacement >= 0.5 * cell_size
    // from the last recorded node to accept a new NodeRecord. Defends against
    // the same intersection being re-detected after a brief partial view.
    bool distance_ok = true;
    if (last_node_local_x_.has_value() && last_node_local_y_.has_value() &&
        in.local_x_m.has_value() && in.local_y_m.has_value()) {
        const double dx = *in.local_x_m - *last_node_local_x_;
        const double dy = *in.local_y_m - *last_node_local_y_;
        distance_ok = std::sqrt(dx * dx + dy * dy) >= 0.5 * config_.cell_size_m;
    }

    if (!lockout_active && !turn_lockout_active && distance_ok &&
        nodeJustRecorded(in.intersection_decision) && in.node_event.valid) {
        ++intersections_recorded_;
        // Cycle 9: tell composition root to commit tracker advance.
        out.commit_tracker_advance = true;
        // Cache branch state for upcoming handleSnakeRecordNode dwell, so the
        // boundary check reads stable values rather than a stale node_event
        // that becomes invalid in subsequent ticks.
        last_node_grid_branch_mask_ = in.node_event.grid_branch_mask;
        last_node_front_open_ = forwardBranchPresent(in.intersection_decision);
        if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
            last_node_local_x_ = *in.local_x_m;
            last_node_local_y_ = *in.local_y_m;
        }

        // Exit conditions check
        if (registry_ && registry_->gridMarkerCount() >= static_cast<std::size_t>(config_.markers_expected)) {
            transition(GridState::SnakeRecordNode, in.now_s, "record_pre_complete");
            last_node_record_s_ = in.now_s;
            return;
        }

        // Check forward branch availability AFTER node
        if (!forwardBranchPresent(in.intersection_decision) ||
            in.node_event.camera_branch_mask == 0) {
            // boundary reached - transition through RecordNode then StopAtCenter
            transition(GridState::SnakeRecordNode, in.now_s, "record_boundary");
            last_node_record_s_ = in.now_s;
            return;
        }
        transition(GridState::SnakeRecordNode, in.now_s, "record_node");
        last_node_record_s_ = in.now_s;
    }
}

void GridMission::handleSnakeRecordNode(const GridMissionInput& in, GridMissionOutput& out)
{
    // Default: line-follow with deceleration through the node (Cycle 9 Option C).
    out.intent = control::GridControlIntent::LineFollow;
    out.advance_phase = true;  // cap forward speed to forward_speed_advance_mps

    // Cycle 9 marker stability: only accept markers seen for N consecutive frames.
    // Cycle 15: once we have latched a marker for this dwell, hold it even if
    // the camera momentarily loses sight of the marker — otherwise a single
    // frame with mks=0 drops focus_marker_id to -1, intent falls back to
    // LineFollow, and the visual hover effect is lost while dwell continues.
    int focus_marker_id = -1;
    if (marker_hover_active_ && last_recorded_marker_id_ >= 0) {
        focus_marker_id = last_recorded_marker_id_;
    } else if (in.vision) {
        for (const auto& m : in.vision->markers) {
            if (m.id == config_.vertiport_marker_id) continue;
            if (registry_ && registry_->hasId(m.id)) continue;
            auto it = marker_candidate_count_.find(m.id);
            const int seen = (it != marker_candidate_count_.end()) ? it->second : 0;
            if (seen < config_.marker_observation_min_frames) continue;
            focus_marker_id = m.id;
            break;
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
        transition(GridState::SnakeForward, in.now_s, "node_done");
    }
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
    if (snake_stop_stable_count_ >= config_.snake_stop_velocity_consecutive_frames) {
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
        if (in.attitude_yaw_rad.has_value()) {
            const double dir = (pending_turn_dir_ == SnakeTurnDir::Right) ? +1.0 : -1.0;
            yaw_target_rad_ = wrap(*in.attitude_yaw_rad + dir * (M_PI / 2.0));
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
            transition(GridState::SnakeAdvanceOneCell, in.now_s, "turn1_done");
        }
    }
}

void GridMission::handleSnakeAdvanceOneCell(const GridMissionInput& in, GridMissionOutput& out)
{
    // Cycle 12 C: blind forward immediately after the turn so the camera
    // clears the old corridor's line before we try to track again.
    if (in.now_s < snake_post_turn_blind_until_s_) {
        out.intent = control::GridControlIntent::ForwardBlind;
        // Reset the distance-gate anchor so the very first node we accept in
        // the new corridor is honoured even if the LOCAL_NED delta is small.
        if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
            last_node_local_x_ = *in.local_x_m;
            last_node_local_y_ = *in.local_y_m;
        }
        // Hard timeout still applies (snake_advance_timeout_s).
        if (in.now_s - state_entry_s_ > config_.snake_advance_timeout_s) {
            out.last_safety_event = "advance_timeout";
            transition(GridState::EmergencyLand, in.now_s, "advance_timeout");
        }
        return;
    }

    out.intent = control::GridControlIntent::LineFollow;
    out.advance_phase = true;

    const bool turn_lockout = in.now_s - last_turn_complete_s_ < config_.snake_turn_lockout_s;

    if (!turn_lockout &&
        nodeJustRecorded(in.intersection_decision) && in.node_event.valid) {
        ++intersections_recorded_;
        last_node_record_s_ = in.now_s;

        // Cycle 15: commit the column-transition cell to the tracker so the
        // subsequent SnakeForward starts from the new column's first row, not
        // from the old column's last row. Without this, column 2+ commits
        // overwrite column 1 coords in the GCS grid and SnakeForward's
        // LineFollow chases the previous column's residual line.
        out.commit_tracker_advance = true;
        if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
            last_node_local_x_ = *in.local_x_m;
            last_node_local_y_ = *in.local_y_m;
        }

        if (in.attitude_yaw_rad.has_value()) {
            const double dir = (pending_turn_dir_ == SnakeTurnDir::Right) ? +1.0 : -1.0;
            yaw_target_rad_ = wrap(*in.attitude_yaw_rad + dir * (M_PI / 2.0));
        }
        snake_yaw_stable_count_ = 0;
        transition(GridState::SnakeTurn90Again, in.now_s, "cell_reached");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.snake_advance_timeout_s) {
        out.last_safety_event = "advance_timeout";
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
            transition(GridState::SnakeForward, in.now_s, "turn2_done");
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
