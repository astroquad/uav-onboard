#include "mission/GridMission.hpp"

#include <algorithm>
#include <climits>
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
    case GridState::EntryOriginMarkerHover:return "ENTRY_ORIGIN_MARKER_HOVER";
    case GridState::SnakeForward:         return "SNAKE_FORWARD";
    case GridState::SnakeRecordNode:      return "SNAKE_RECORD_NODE";
    case GridState::SnakeLaunchAlign:     return "SNAKE_LAUNCH_ALIGN";
    case GridState::SnakeStopAtCenter:    return "SNAKE_STOP_AT_CENTER";
    case GridState::SnakeTurn90:          return "SNAKE_TURN_90";
    case GridState::SnakeAdvanceOneCell:  return "SNAKE_ADVANCE_ONE_CELL";
    case GridState::SnakeTurn90Again:     return "SNAKE_TURN_90_AGAIN";
    case GridState::TurnNodeMarkerHover:  return "TURN_NODE_MARKER_HOVER";
    case GridState::SnakeComplete:        return "SNAKE_COMPLETE";
    case GridState::RevisitInit:          return "REVISIT_INIT";
    case GridState::RevisitForward:       return "REVISIT_FORWARD";
    case GridState::RevisitStopAtTurn:    return "REVISIT_STOP_AT_TURN";
    case GridState::RevisitTurn90:        return "REVISIT_TURN_90";
    case GridState::RevisitMarkerHover:   return "REVISIT_MARKER_HOVER";
    case GridState::RevisitComplete:      return "REVISIT_COMPLETE";
    case GridState::ReturnHomeInit:       return "RETURN_HOME_INIT";
    case GridState::ReturnHomeForward:    return "RETURN_HOME_FORWARD";
    case GridState::ReturnHomeStopAtTurn: return "RETURN_HOME_STOP_AT_TURN";
    case GridState::ReturnHomeTurn90:     return "RETURN_HOME_TURN_90";
    case GridState::ReturnHomeAlignOrigin:return "RETURN_HOME_ALIGN_ORIGIN";
    case GridState::ReturnHomeFaceSouth:  return "RETURN_HOME_FACE_SOUTH";
    case GridState::ReturnVertiportForward:return "RETURN_VERTIPORT_FORWARD";
    case GridState::ReturnVertiportMarkerHover:return "RETURN_VERTIPORT_MARKER_HOVER";
    case GridState::MissionComplete:      return "MISSION_COMPLETE";
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
    active_vertiport_marker_id_ = -1;
    candidate_vertiport_id_ = -1;
    candidate_vertiport_count_ = 0;
    pending_synth_events_.clear();
    entry_center_stable_count_ = 0;
    entry_forward_start_frame_seq_ = 0;
    snake_stop_stable_count_ = 0;
    snake_stop_settle_entry_s_ = -1.0;
    snake_yaw_stable_count_ = 0;
    snake_launch_align_stable_count_ = 0;
    origin_latched_ = false;
    pending_turn_dir_ = SnakeTurnDir::Unknown;
    pending_post_turn_heading_ = GridHeading::Unknown;
    pending_post_hover_state_ = GridState::Idle;
    pending_post_hover_marker_id_ = -1;
    pending_post_hover_yaw_target_rad_ = 0.0;
    snake_turn_first_done_ = false;
    marker_hover_active_ = false;
    marker_hover_start_s_ = -1.0;
    marker_hover_centered_count_ = 0;
    last_recorded_marker_id_ = -1;
    completion_hover_marker_id_ = -1;
    entry_origin_marker_id_ = -1;
    resetRevisitPlan();
    resetReturnHomePlan();
    grid_map_finalized_ = false;
    if (registry_) registry_->reset();
    if (tracker_) tracker_->resetLocalOrigin();
    if (altitude_) altitude_->reset();
    if (snake_) snake_->reset();
    last_node_local_x_.reset();
    last_node_local_y_.reset();
    hop_start_local_x_.reset();
    hop_start_local_y_.reset();
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

std::optional<onboard::vision::MarkerObservation> GridMission::findAnyMarker(
    const onboard::vision::VisionResult* vis) const
{
    if (!vis || vis->markers.empty()) return std::nullopt;
    return vis->markers.front();
}

int GridMission::effectiveVertiportMarkerId() const
{
    return active_vertiport_marker_id_ >= 0
        ? active_vertiport_marker_id_
        : config_.vertiport_marker_id;
}

int GridMission::tryRegisterCurrentCellMarker(GridCoord coord,
                                               const GridMissionInput& in,
                                               const char* context)
{
    if (!registry_) return -1;
    const int stable_id = marker_window_.bestStableId();
    if (stable_id < 0) return -1;
    if (stable_id == effectiveVertiportMarkerId()) return -1;
    if (registry_->hasId(stable_id)) {
        // Already known marker — no fresh registration, no hover needed.
        return -1;
    }
    float orientation = 0.0f;
    if (auto obs = findMarker(in.vision, stable_id)) {
        orientation = obs->orientation_deg;
    }
    const bool was_new = registry_->observe(stable_id, coord, /*grid_coord_valid=*/true,
                                            orientation, in.timestamp_ms, in.frame_seq);
    if (was_new) {
        std::cerr << "[marker-commit] id=" << stable_id
                  << " coord=(" << coord.x << "," << coord.y << ")"
                  << " (" << (context ? context : "grid-cell") << ")\n";
    }
    return stable_id;
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

    // Forward hops now stay in ForwardBlind for the full cell. Yaw is always
    // mission-locked in the mapper; we only pass lateral line-center error
    // through when the current view is a confident straight corridor. Near
    // intersections and side branches, zero the line input so ForwardBlind
    // becomes pure yaw-locked dead reckoning again.
    const bool launch_state = state_ == GridState::SnakeLaunchAlign;
    const bool snake_state =
        (state_ == GridState::SnakeForward ||
         state_ == GridState::SnakeAdvanceOneCell ||
         state_ == GridState::RevisitForward ||
         state_ == GridState::ReturnHomeForward);

    if (launch_state) {
        if (!forwardBranchPresent(in.intersection_decision) && !last_node_front_open_) {
            out.line_angle_error_rad = 0.0;
            out.line_center_error_norm = 0.0;
            out.line_detected = false;
        }
        return;
    }

    if (!snake_state) {
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

void GridMission::beginMarkerHover(int marker_id, double now_s)
{
    if (!marker_hover_active_ ||
        last_recorded_marker_id_ != marker_id ||
        marker_hover_start_s_ < 0.0) {
        marker_hover_start_s_ = now_s;
        marker_hover_centered_count_ = 0;
    }
    marker_hover_active_ = true;
    last_recorded_marker_id_ = marker_id;
}

void GridMission::clearMarkerHover()
{
    marker_hover_active_ = false;
    marker_hover_start_s_ = -1.0;
    marker_hover_centered_count_ = 0;
}

bool GridMission::updateMarkerHoverCenterGate(const GridMissionOutput& out)
{
    const double tol = std::max(0.0, config_.marker_hover_center_tolerance_norm);
    const bool centered =
        out.marker_detected &&
        std::abs(out.marker_center_error_x_norm) <= tol &&
        std::abs(out.marker_center_error_y_norm) <= tol;
    if (centered) {
        ++marker_hover_centered_count_;
    } else {
        marker_hover_centered_count_ = 0;
    }
    return marker_hover_centered_count_ >=
        std::max(1, config_.marker_hover_center_stable_frames);
}

bool GridMission::markerHoverComplete(const GridMissionOutput& out, double now_s)
{
    const double start_s = marker_hover_start_s_ >= 0.0
        ? marker_hover_start_s_
        : state_entry_s_;
    const double elapsed_s = now_s - start_s;
    (void)updateMarkerHoverCenterGate(out);
    return elapsed_s >= std::max(0.0, config_.snake_marker_hover_s);
}

bool GridMission::hasPendingGridMarkerHint(const GridMissionInput& in) const
{
    const int vertiport_id = effectiveVertiportMarkerId();
    auto pending_grid_marker = [&](int id) {
        if (id < 0 || id == vertiport_id) return false;
        return !registry_ || !registry_->hasId(id);
    };

    if (in.vision) {
        for (const auto& marker : in.vision->markers) {
            if (pending_grid_marker(marker.id)) {
                return true;
            }
        }
    }
    for (int id : marker_window_.ids) {
        if (pending_grid_marker(id)) {
            return true;
        }
    }
    return false;
}

bool GridMission::shouldPassThroughRegularNode(const GridMissionInput& in) const
{
    if (!config_.snake_passthrough_regular_nodes) return false;
    if (!in.node_event.valid) return false;
    if (!isGridNodeType(in.node_event.topology) &&
        !isGridNodeType(in.intersection_decision.accepted_type)) {
        return false;
    }
    if (hasPendingGridMarkerHint(in)) return false;
    if (in.intersection_decision.required_turn || in.intersection_decision.turn_candidate) {
        return false;
    }
    if (!forwardBranchPresent(in.intersection_decision)) return false;
    if (in.node_event.camera_branch_mask == 0 || in.node_event.grid_branch_mask == 0) {
        return false;
    }
    return true;
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
        state_ != GridState::MissionComplete &&
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
        const int vertiport_id_runtime = effectiveVertiportMarkerId();
        if (input.vision) {
            for (const auto& m : input.vision->markers) {
                if (m.id == vertiport_id_runtime) continue;
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
    case GridState::EntryOriginMarkerHover:handleEntryOriginMarkerHover(input, out); break;
    case GridState::SnakeForward:         handleSnakeForward(input, out); break;
    case GridState::SnakeRecordNode:      handleSnakeRecordNode(input, out); break;
    case GridState::SnakeLaunchAlign:     handleSnakeLaunchAlign(input, out); break;
    case GridState::SnakeStopAtCenter:    handleSnakeStopAtCenter(input, out); break;
    case GridState::SnakeTurn90:          handleSnakeTurn90(input, out); break;
    case GridState::SnakeAdvanceOneCell:  handleSnakeAdvanceOneCell(input, out); break;
    case GridState::SnakeTurn90Again:     handleSnakeTurn90Again(input, out); break;
    case GridState::TurnNodeMarkerHover:  handleTurnNodeMarkerHover(input, out); break;
    case GridState::SnakeComplete:        handleSnakeComplete(input, out); break;
    case GridState::RevisitInit:          handleRevisitInit(input, out); break;
    case GridState::RevisitForward:       handleRevisitForward(input, out); break;
    case GridState::RevisitStopAtTurn:    handleRevisitStopAtTurn(input, out); break;
    case GridState::RevisitTurn90:        handleRevisitTurn90(input, out); break;
    case GridState::RevisitMarkerHover:   handleRevisitMarkerHover(input, out); break;
    case GridState::RevisitComplete:      handleRevisitComplete(input, out); break;
    case GridState::ReturnHomeInit:       handleReturnHomeInit(input, out); break;
    case GridState::ReturnHomeForward:    handleReturnHomeForward(input, out); break;
    case GridState::ReturnHomeStopAtTurn: handleReturnHomeStopAtTurn(input, out); break;
    case GridState::ReturnHomeTurn90:     handleReturnHomeTurn90(input, out); break;
    case GridState::ReturnHomeAlignOrigin:handleReturnHomeAlignOrigin(input, out); break;
    case GridState::ReturnHomeFaceSouth:  handleReturnHomeFaceSouth(input, out); break;
    case GridState::ReturnVertiportForward:handleReturnVertiportForward(input, out); break;
    case GridState::ReturnVertiportMarkerHover:handleReturnVertiportMarkerHover(input, out); break;
    case GridState::MissionComplete:      handleMissionComplete(input, out); break;
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
    if (grid_pose_visible_ &&
        last_node_local_x_.has_value() && last_node_local_y_.has_value() &&
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
    populateRevisitTelemetry(out);
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
}

double GridMission::hopDistance(const GridMissionInput& in) const
{
    if (!hop_start_local_x_.has_value() || !hop_start_local_y_.has_value()) return 0.0;
    if (!in.local_x_m.has_value() || !in.local_y_m.has_value()) return 0.0;
    const double dx = *in.local_x_m - *hop_start_local_x_;
    const double dy = *in.local_y_m - *hop_start_local_y_;
    return std::sqrt(dx * dx + dy * dy);
}

bool GridMission::isRevisitState() const
{
    switch (state_) {
    case GridState::RevisitInit:
    case GridState::RevisitForward:
    case GridState::RevisitStopAtTurn:
    case GridState::RevisitTurn90:
    case GridState::RevisitMarkerHover:
    case GridState::RevisitComplete:
        return true;
    default:
        return false;
    }
}

bool GridMission::isReturnState() const
{
    switch (state_) {
    case GridState::ReturnHomeInit:
    case GridState::ReturnHomeForward:
    case GridState::ReturnHomeStopAtTurn:
    case GridState::ReturnHomeTurn90:
    case GridState::ReturnHomeAlignOrigin:
    case GridState::ReturnHomeFaceSouth:
    case GridState::ReturnVertiportForward:
    case GridState::ReturnVertiportMarkerHover:
        return true;
    default:
        return false;
    }
}

const char* GridMission::returnPhase() const
{
    switch (state_) {
    case GridState::ReturnHomeInit:
    case GridState::ReturnHomeForward:
    case GridState::ReturnHomeStopAtTurn:
    case GridState::ReturnHomeTurn90:
        return "grid_home";
    case GridState::ReturnHomeAlignOrigin:
        return "origin_align";
    case GridState::ReturnHomeFaceSouth:
        return "face_south";
    case GridState::ReturnVertiportForward:
        return "vertiport_forward";
    case GridState::ReturnVertiportMarkerHover:
        return "vertiport_hover";
    default:
        return "none";
    }
}

void GridMission::populateRevisitTelemetry(GridMissionOutput& out) const
{
    out.revisit_active = isRevisitState();
    out.return_active = isReturnState();
    out.return_phase = returnPhase();
    out.grid_map_finalized = grid_map_finalized_;
    out.grid_pose_visible = grid_pose_visible_;
    out.vertiport_return_active =
        state_ == GridState::ReturnVertiportForward ||
        state_ == GridState::ReturnVertiportMarkerHover;
    out.vertiport_acquired = vertiport_acquired_;
    out.landing_success = landing_success_;
    out.mission_complete = mission_complete_;
    out.revisit_order = revisitOrderName(config_.revisit_order);
    out.revisit_target_id = revisit_current_marker_id_;
    int remaining = 0;
    if (revisit_route_ready_ && revisit_leg_index_ < revisit_legs_.size()) {
        remaining = static_cast<int>(revisit_legs_.size() - revisit_leg_index_);
    }
    out.revisit_remaining = remaining;
}

void GridMission::resetReturnHomePlan()
{
    return_home_leg_ = {};
    return_segment_index_ = 0;
    return_cells_remaining_in_segment_ = 0;
    return_current_coord_ = {};
    return_current_heading_ = GridHeading::Unknown;
    return_desired_heading_ = GridHeading::Unknown;
    return_turn_step_heading_ = GridHeading::Unknown;
    return_home_route_ready_ = false;
    grid_pose_visible_ = true;
    vertiport_acquired_ = false;
    landing_success_ = false;
    mission_complete_ = false;
}

void GridMission::resetRevisitPlan()
{
    revisit_legs_.clear();
    revisit_leg_index_ = 0;
    revisit_segment_index_ = 0;
    revisit_cells_remaining_in_segment_ = 0;
    revisit_current_coord_ = {};
    revisit_current_heading_ = GridHeading::Unknown;
    revisit_desired_heading_ = GridHeading::Unknown;
    revisit_turn_step_heading_ = GridHeading::Unknown;
    revisit_yaw_reference_heading_ = GridHeading::Unknown;
    revisit_yaw_reference_rad_ = 0.0;
    revisit_current_marker_id_ = -1;
    revisit_route_ready_ = false;
}

bool GridMission::buildRevisitPlan()
{
    if (!tracker_ || !registry_ || config_.revisit_order == RevisitOrder::None) {
        return false;
    }
    std::vector<MarkerRevisitTarget> targets;
    for (const auto& record : registry_->records()) {
        if (!record.grid_coord_valid || record.revisited) {
            continue;
        }
        targets.push_back(MarkerRevisitTarget{record.aruco_id, record.grid_coord});
    }
    revisit_current_coord_ = tracker_->currentCoord();
    revisit_current_heading_ = tracker_->currentHeading();
    revisit_yaw_reference_heading_ = revisit_current_heading_;
    revisit_yaw_reference_rad_ = yaw_align_target_rad_;
    revisit_legs_ = revisit_planner_.buildPlan(
        revisit_current_coord_, revisit_current_heading_, targets, config_.revisit_order);
    revisit_leg_index_ = 0;
    revisit_segment_index_ = 0;
    revisit_cells_remaining_in_segment_ = 0;
    revisit_current_marker_id_ = -1;
    revisit_route_ready_ = true;
    return !revisit_legs_.empty();
}

GridHeading GridMission::currentRevisitSegmentHeading() const
{
    if (revisit_leg_index_ >= revisit_legs_.size()) {
        return GridHeading::Unknown;
    }
    const auto& leg = revisit_legs_[revisit_leg_index_];
    if (revisit_segment_index_ >= leg.segments.size()) {
        return GridHeading::Unknown;
    }
    return leg.segments[revisit_segment_index_].heading;
}

GridHeading GridMission::nextTurnStepHeading(GridHeading from, GridHeading to) const
{
    if (from == GridHeading::Unknown || from == to) {
        return to;
    }
    if (turnRight(from) == to || turnLeft(from) != to) {
        return turnRight(from);
    }
    return turnLeft(from);
}

double GridMission::yawForHeading(GridHeading heading) const
{
    const GridHeading reference_heading =
        revisit_yaw_reference_heading_ != GridHeading::Unknown
            ? revisit_yaw_reference_heading_
            : (tracker_ ? tracker_->currentHeading() : GridHeading::Unknown);
    const double reference_yaw =
        revisit_yaw_reference_heading_ != GridHeading::Unknown
            ? revisit_yaw_reference_rad_
            : yaw_align_target_rad_;
    auto index = [](GridHeading h) {
        switch (h) {
        case GridHeading::North: return 0;
        case GridHeading::East:  return 1;
        case GridHeading::South: return 2;
        case GridHeading::West:  return 3;
        case GridHeading::Unknown: return -1;
        }
        return -1;
    };
    const int ref = index(reference_heading);
    const int dst = index(heading);
    if (ref < 0 || dst < 0) {
        return reference_yaw;
    }
    int delta = (dst - ref + 4) % 4;
    if (delta == 3) {
        delta = -1;
    }
    return wrap(reference_yaw + static_cast<double>(delta) * (M_PI / 2.0));
}

bool GridMission::startCurrentRevisitLeg(const GridMissionInput& in,
                                         GridMissionOutput& out)
{
    (void)out;
    if (revisit_leg_index_ >= revisit_legs_.size()) {
        revisit_current_marker_id_ = -1;
        transition(GridState::RevisitComplete, in.now_s, "revisit_done");
        return false;
    }

    const auto& leg = revisit_legs_[revisit_leg_index_];
    revisit_current_marker_id_ = leg.marker_id;
    if (leg.segments.empty()) {
        transition(GridState::RevisitMarkerHover, in.now_s, "revisit_at_marker");
        return true;
    }

    if (revisit_segment_index_ >= leg.segments.size()) {
        revisit_segment_index_ = 0;
    }
    revisit_cells_remaining_in_segment_ =
        leg.segments[revisit_segment_index_].cells;
    revisit_desired_heading_ = leg.segments[revisit_segment_index_].heading;
    if (revisit_current_heading_ != revisit_desired_heading_) {
        snake_stop_stable_count_ = 0;
        snake_stop_settle_entry_s_ = -1.0;
        transition(GridState::RevisitStopAtTurn, in.now_s, "revisit_turn_needed");
        return true;
    }

    armHopStart(in);
    transition(GridState::RevisitForward, in.now_s, "revisit_forward");
    return true;
}

bool GridMission::advanceRevisitNode(const GridMissionInput& in)
{
    const GridHeading heading = currentRevisitSegmentHeading();
    if (!tracker_ || heading == GridHeading::Unknown ||
        revisit_cells_remaining_in_segment_ <= 0) {
        return false;
    }
    revisit_current_coord_ = tracker_->advance(revisit_current_coord_, heading);
    revisit_current_heading_ = heading;
    tracker_->setCurrentPose(revisit_current_coord_, revisit_current_heading_);
    --revisit_cells_remaining_in_segment_;
    last_node_record_s_ = in.now_s;
    if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
        last_node_local_x_ = *in.local_x_m;
        last_node_local_y_ = *in.local_y_m;
    }
    return true;
}

bool GridMission::finishRevisitSegmentOrLeg(const GridMissionInput& in,
                                            GridMissionOutput& out)
{
    (void)out;
    if (revisit_leg_index_ >= revisit_legs_.size()) {
        transition(GridState::RevisitComplete, in.now_s, "revisit_done");
        return true;
    }
    const auto& leg = revisit_legs_[revisit_leg_index_];
    if (revisit_cells_remaining_in_segment_ > 0) {
        armHopStart(in);
        transition(GridState::RevisitForward, in.now_s, "revisit_continue");
        return true;
    }

    ++revisit_segment_index_;
    if (revisit_segment_index_ >= leg.segments.size()) {
        transition(GridState::RevisitMarkerHover, in.now_s, "revisit_marker");
        return true;
    }

    revisit_cells_remaining_in_segment_ =
        leg.segments[revisit_segment_index_].cells;
    revisit_desired_heading_ = leg.segments[revisit_segment_index_].heading;
    if (revisit_current_heading_ != revisit_desired_heading_) {
        snake_stop_stable_count_ = 0;
        snake_stop_settle_entry_s_ = -1.0;
        transition(GridState::RevisitStopAtTurn, in.now_s, "revisit_turn_needed");
        return true;
    }
    armHopStart(in);
    transition(GridState::RevisitForward, in.now_s, "revisit_continue");
    return true;
}

bool GridMission::buildReturnHomePlan()
{
    if (!tracker_) {
        return false;
    }
    return_current_coord_ = tracker_->currentCoord();
    return_current_heading_ = tracker_->currentHeading();
    if (return_current_heading_ == GridHeading::Unknown) {
        return_current_heading_ = revisit_current_heading_ != GridHeading::Unknown
            ? revisit_current_heading_
            : GridHeading::North;
    }
    if (revisit_yaw_reference_heading_ == GridHeading::Unknown) {
        revisit_yaw_reference_heading_ = return_current_heading_;
        revisit_yaw_reference_rad_ = yaw_align_target_rad_;
    }

    return_home_leg_ = revisit_planner_.buildLeg(
        return_current_coord_, return_current_heading_, GridCoord{0, 0}, -1);
    return_segment_index_ = 0;
    return_cells_remaining_in_segment_ = 0;
    return_desired_heading_ = GridHeading::Unknown;
    return_turn_step_heading_ = GridHeading::Unknown;
    return_home_route_ready_ = true;
    grid_pose_visible_ = true;
    return true;
}

GridHeading GridMission::currentReturnSegmentHeading() const
{
    if (return_segment_index_ >= return_home_leg_.segments.size()) {
        return GridHeading::Unknown;
    }
    return return_home_leg_.segments[return_segment_index_].heading;
}

bool GridMission::startCurrentReturnLeg(const GridMissionInput& in,
                                        GridMissionOutput& out)
{
    (void)out;
    if (!return_home_route_ready_ && !buildReturnHomePlan()) {
        last_safety_event_ = "return_home_plan_failed";
        transition(GridState::EmergencyLand, in.now_s, "return_home_plan_failed");
        return false;
    }

    if (return_home_leg_.segments.empty()) {
        entry_center_stable_count_ = 0;
        transition(GridState::ReturnHomeAlignOrigin, in.now_s, "return_at_origin");
        return true;
    }

    if (return_segment_index_ >= return_home_leg_.segments.size()) {
        return_segment_index_ = 0;
    }
    return_cells_remaining_in_segment_ =
        return_home_leg_.segments[return_segment_index_].cells;
    return_desired_heading_ =
        return_home_leg_.segments[return_segment_index_].heading;
    if (return_current_heading_ != return_desired_heading_) {
        snake_stop_stable_count_ = 0;
        snake_stop_settle_entry_s_ = -1.0;
        transition(GridState::ReturnHomeStopAtTurn, in.now_s, "return_turn_needed");
        return true;
    }

    armHopStart(in);
    transition(GridState::ReturnHomeForward, in.now_s, "return_forward");
    return true;
}

bool GridMission::advanceReturnNode(const GridMissionInput& in)
{
    const GridHeading heading = currentReturnSegmentHeading();
    if (!tracker_ || heading == GridHeading::Unknown ||
        return_cells_remaining_in_segment_ <= 0) {
        return false;
    }
    return_current_coord_ = tracker_->advance(return_current_coord_, heading);
    return_current_heading_ = heading;
    tracker_->setCurrentPose(return_current_coord_, return_current_heading_);
    --return_cells_remaining_in_segment_;
    last_node_record_s_ = in.now_s;
    if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
        last_node_local_x_ = *in.local_x_m;
        last_node_local_y_ = *in.local_y_m;
    }
    return true;
}

bool GridMission::finishReturnSegmentOrLeg(const GridMissionInput& in,
                                           GridMissionOutput& out)
{
    (void)out;
    if (return_cells_remaining_in_segment_ > 0) {
        armHopStart(in);
        transition(GridState::ReturnHomeForward, in.now_s, "return_continue");
        return true;
    }

    ++return_segment_index_;
    if (return_segment_index_ >= return_home_leg_.segments.size()) {
        entry_center_stable_count_ = 0;
        transition(GridState::ReturnHomeAlignOrigin, in.now_s, "return_origin_reached");
        return true;
    }

    return_cells_remaining_in_segment_ =
        return_home_leg_.segments[return_segment_index_].cells;
    return_desired_heading_ =
        return_home_leg_.segments[return_segment_index_].heading;
    if (return_current_heading_ != return_desired_heading_) {
        snake_stop_stable_count_ = 0;
        snake_stop_settle_entry_s_ = -1.0;
        transition(GridState::ReturnHomeStopAtTurn, in.now_s, "return_turn_needed");
        return true;
    }
    armHopStart(in);
    transition(GridState::ReturnHomeForward, in.now_s, "return_continue");
    return true;
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

    // Cycle 24: dynamic vertiport id. Until a stable candidate is latched
    // into `active_vertiport_marker_id_`, accept any marker as the vertiport
    // candidate. Once latched, the rest of the mission keys off that id via
    // effectiveVertiportMarkerId().
    std::optional<onboard::vision::MarkerObservation> marker;
    if (active_vertiport_marker_id_ >= 0) {
        marker = findMarker(in.vision, active_vertiport_marker_id_);
    } else {
        marker = findAnyMarker(in.vision);
        if (marker.has_value()) {
            if (marker->id == candidate_vertiport_id_) {
                ++candidate_vertiport_count_;
            } else {
                candidate_vertiport_id_ = marker->id;
                candidate_vertiport_count_ = 1;
            }
            if (candidate_vertiport_count_ >= config_.vertiport_marker_stable_frames) {
                active_vertiport_marker_id_ = candidate_vertiport_id_;
                marker_window_.clear();
            }
        } else {
            candidate_vertiport_id_ = -1;
            candidate_vertiport_count_ = 0;
        }
    }
    if (marker.has_value()) {
        populateMarkerInputs(in, marker->id, out);
        if (active_vertiport_marker_id_ >= 0 &&
            marker->id == active_vertiport_marker_id_) {
            ++vertiport_marker_stable_count_;
            if (registry_) {
                registry_->observe(marker->id, GridCoord{}, /*grid_coord_valid=*/false,
                                   marker->orientation_deg, in.timestamp_ms, in.frame_seq);
            }
            out.vertiport_verified = true;
        } else {
            vertiport_marker_stable_count_ = 0;
        }
    } else {
        vertiport_marker_stable_count_ = 0;
        out.marker_detected = false;
    }
    out.vertiport_verified = active_vertiport_marker_id_ >= 0;

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
        active_vertiport_marker_id_ >= 0 &&
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
        const int origin_marker_id =
            tryRegisterCurrentCellMarker(GridCoord{0, 0}, in, "origin");
        marker_window_.clear();
        if (origin_marker_id >= 0) {
            entry_origin_marker_id_ = origin_marker_id;
            beginMarkerHover(origin_marker_id, in.now_s);
            transition(GridState::EntryOriginMarkerHover,
                       in.now_s,
                       "origin_marker_hover");
            return;
        }
        transition(GridState::SnakeLaunchAlign, in.now_s, "origin_centered");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.entry_center_timeout_s) {
        last_safety_event_ = "entry_center_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "entry_center_timeout");
    }
}

void GridMission::handleEntryOriginMarkerHover(const GridMissionInput& in,
                                               GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::MarkerHover;
    out.target_yaw_rad = yaw_align_target_rad_;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.vertiport_verified = true;
    out.advance_phase = false;

    const int focus_id = entry_origin_marker_id_;
    if (focus_id >= 0) {
        beginMarkerHover(focus_id, in.now_s);
        populateMarkerInputs(in, focus_id, out);
        if (registry_) {
            auto obs = findMarker(in.vision, focus_id);
            const float orientation = obs.has_value() ? obs->orientation_deg : 0.0f;
            registry_->observe(focus_id,
                               GridCoord{0, 0},
                               /*grid_coord_valid=*/true,
                               orientation,
                               in.timestamp_ms,
                               in.frame_seq);
        }
    }

    const bool hover_done = focus_id >= 0
        ? markerHoverComplete(out, in.now_s)
        : (in.now_s - state_entry_s_ >= config_.snake_marker_hover_s);
    if (!hover_done) {
        return;
    }

    clearMarkerHover();
    marker_window_.clear();
    last_recorded_marker_id_ = -1;
    entry_origin_marker_id_ = -1;
    snake_launch_align_stable_count_ = 0;
    if (decision_engine_) {
        decision_engine_->reset();
    }
    transition(GridState::SnakeLaunchAlign, in.now_s, "origin_marker_done");
}

void GridMission::handleSnakeForward(const GridMissionInput& in, GridMissionOutput& out)
{
    // Hop-to-hop. From the last committed node, fly body-forward for the
    // whole cell with yaw locked to the column target. ForwardBlind may borrow
    // only lateral vy from the line controller when populateLineInputs sees a
    // confident straight corridor, so it recenters without ever granting line
    // angle authority over yaw.
    //
    // The next node arrival is detected by intersection.valid AND a minimum
    // hop distance gate so the just-departed node's residual lookahead does
    // not retrigger.

    if (!hop_start_local_x_.has_value()) {
        armHopStart(in);
    }
    out.target_altitude_m = config_.cruise_altitude_m;

    const double distance = hopDistance(in);

    const bool post_turn_blind = in.now_s < snake_post_turn_blind_until_s_;
    out.target_yaw_rad = yaw_align_target_rad_;
    out.intent = control::GridControlIntent::ForwardBlind;

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
        const GridHeading current_heading =
            tracker_ ? tracker_->currentHeading() : GridHeading::North;
        last_node_front_open_ = forwardBranchPresent(in.intersection_decision);
        last_node_grid_branch_mask_ = rotateCameraBranchMaskToGrid(
            in.intersection_decision.accepted_branch_mask, current_heading);

        // Cycle 23: also commit the boundary node here so the last node of
        // every column reaches the GCS map. IntersectionDecision routes
        // required_turn intersections directly to TurnConfirm/TurnReady,
        // skipping NodeRecord, so the regular arrival path below is gated
        // off. event_ready is still set when cy hits the node band, so
        // tracker peek typically returns valid=true at this exact frame;
        // when it does not (cy entered turn_zone only), synthesise an event
        // describing the boundary node so commitAdvance still fires.
        ++intersections_recorded_;
        out.commit_tracker_advance = true;
        if (in.node_event.valid) {
            // peek event already describes the boundary node; just commit it.
        } else if (tracker_) {
            GridNodeEvent ev;
            ev.valid = true;
            ev.arrival_heading = current_heading;
            ev.local_coord = tracker_->advance(tracker_->currentCoord(), current_heading);
            ev.topology = in.intersection_decision.accepted_type;
            ev.camera_branch_mask = in.intersection_decision.accepted_branch_mask;
            ev.grid_branch_mask = last_node_grid_branch_mask_;
            ev.origin_local_only = true;
            out.synthetic_commit_event = ev;
        }
        if (in.local_x_m.has_value() && in.local_y_m.has_value()) {
            last_node_local_x_ = *in.local_x_m;
            last_node_local_y_ = *in.local_y_m;
        }
        last_node_record_s_ = in.now_s;

        // Cycle 25: register a marker stable in marker_window_ against the
        // boundary cell coord. handleSnakeRecordNode (which normally handles
        // marker hover + registration) is skipped on the boundary watchdog
        // path; without this, any marker placed on a column-end cell would
        // never reach the registry.
        const GridCoord boundary_coord = tracker_
            ? tracker_->advance(tracker_->currentCoord(), current_heading)
            : GridCoord{};
        const int hover_id = tryRegisterCurrentCellMarker(boundary_coord, in);

        out.last_safety_event = "";
        // Cycle 26: if a new marker was registered at this boundary cell,
        // route through TurnNodeMarkerHover for the full marker dwell before
        // continuing to SnakeStopAtCenter. Plain turn nodes without markers
        // still go straight to turn-direction settling.
        if (hover_id >= 0) {
            pending_post_hover_state_ = GridState::SnakeStopAtCenter;
            pending_post_hover_marker_id_ = hover_id;
            pending_post_hover_yaw_target_rad_ = yaw_align_target_rad_;
            last_recorded_marker_id_ = hover_id;
            transition(GridState::TurnNodeMarkerHover, in.now_s, "boundary_marker_hover");
        } else {
            transition(GridState::SnakeStopAtCenter, in.now_s, "boundary_watchdog");
            snake_stop_stable_count_ = 0;
        }
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
        if (shouldPassThroughRegularNode(in)) {
            last_node_record_s_ = in.now_s;
            armHopStart(in);
            out.intent = control::GridControlIntent::ForwardBlind;
            out.reason = "passthrough_node";
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
            stable_id != effectiveVertiportMarkerId() &&
            (!registry_ || !registry_->hasId(stable_id))) {
            focus_marker_id = stable_id;
        }
    }
    if (focus_marker_id >= 0) {
        beginMarkerHover(focus_marker_id, in.now_s);
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

    const bool boundary_record =
        !last_node_front_open_ || last_node_grid_branch_mask_ == 0;
    const double record_dwell_s = boundary_record
        ? config_.snake_boundary_record_dwell_s
        : config_.snake_record_dwell_s;
    const bool dwell_done = marker_hover_active_
        ? markerHoverComplete(out, in.now_s)
        : (in.now_s - state_entry_s_ >= record_dwell_s);
    if (dwell_done) {
        const int hover_marker_id = marker_hover_active_
            ? last_recorded_marker_id_
            : -1;
        clearMarkerHover();
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
            completion_hover_marker_id_ = hover_marker_id;
            // Cycle 24: close out the rest of the current column so the grid
            // model onboard (and the GCS render) is a complete rectangle.
            synthesizeRemainingColumnNodes();
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
    // Advance across the one-cell column transition using the same yaw-locked
    // ForwardBlind + lateral vy correction model as SnakeForward.
    if (!hop_start_local_x_.has_value()) {
        armHopStart(in);
    }
    out.target_altitude_m = config_.cruise_altitude_m;

    const double distance = hopDistance(in);

    const bool post_turn_blind = in.now_s < snake_post_turn_blind_until_s_;
    out.target_yaw_rad = yaw_target_rad_;
    out.intent = control::GridControlIntent::ForwardBlind;

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

        // Cycle 25: register marker at the column-transition cell so markers
        // placed on second-turn cells are not silently missed. The
        // all_markers check stays out of this path on purpose — it only
        // fires from handleSnakeRecordNode (N/S heading) so the synthesize
        // pass that closes the column never runs on a mid-transition E/W
        // heading.
        const int hover_id = tryRegisterCurrentCellMarker(in.node_event.local_coord, in);

        const double connector_yaw_target_rad = yaw_target_rad_;

        // Cycle 17: second-turn target also derives from yaw_target_rad_
        // (which itself is yaw_align_target_rad_ + first dir·π/2), so the
        // new column's reference is exactly the original ± π — drift-free.
        {
            const double dir = (pending_turn_dir_ == SnakeTurnDir::Right) ? +1.0 : -1.0;
            yaw_target_rad_ = wrap(yaw_target_rad_ + dir * (M_PI / 2.0));
        }
        snake_yaw_stable_count_ = 0;
        // Cycle 26: hover for the full marker dwell first if this cell had a
        // marker. pending_post_hover_state_ + yaw_target_rad_ are latched
        // before the hover so TurnNodeMarkerHover can resume the second turn.
        if (hover_id >= 0) {
            pending_post_hover_state_ = GridState::SnakeTurn90Again;
            pending_post_hover_marker_id_ = hover_id;
            pending_post_hover_yaw_target_rad_ = connector_yaw_target_rad;
            last_recorded_marker_id_ = hover_id;
            transition(GridState::TurnNodeMarkerHover, in.now_s, "advance_marker_hover");
        } else {
            transition(GridState::SnakeTurn90Again, in.now_s, "cell_reached");
        }
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

void GridMission::handleTurnNodeMarkerHover(const GridMissionInput& in,
                                             GridMissionOutput& out)
{
    // Cycle 26: 3-second marker hover at a turn cell before continuing into
    // the boundary stop (first turn) or the second 90° rotation. Matches
    // SnakeRecordNode's marker hover for regular cells. pending_post_hover_
    // state_ controls where to go after hover. yaw_target_rad_ (for the
    // SnakeTurn90Again path) and snake_stop_stable_count_ (for the
    // SnakeStopAtCenter path) were already latched before the transition.
    out.intent = control::GridControlIntent::MarkerHover;
    out.target_yaw_rad = pending_post_hover_yaw_target_rad_;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.advance_phase = false;

    const int focus_id = pending_post_hover_marker_id_ >= 0
        ? pending_post_hover_marker_id_
        : last_recorded_marker_id_;
    if (focus_id >= 0) {
        beginMarkerHover(focus_id, in.now_s);
        populateMarkerInputs(in, focus_id, out);
        // Re-observe each tick so registry.last_seen_ms tracks current
        // observations (mirrors handleSnakeRecordNode's behaviour).
        if (registry_) {
            auto obs = findMarker(in.vision, focus_id);
            const float orientation = obs.has_value() ? obs->orientation_deg : 0.0f;
            registry_->observe(focus_id,
                               tracker_ ? tracker_->currentCoord() : GridCoord{},
                               /*grid_coord_valid=*/true,
                               orientation,
                               in.timestamp_ms,
                               in.frame_seq);
        }
    }

    const bool hover_done = focus_id >= 0
        ? markerHoverComplete(out, in.now_s)
        : (in.now_s - state_entry_s_ >= config_.snake_marker_hover_s);
    if (hover_done) {
        const GridState next = pending_post_hover_state_ != GridState::Idle
            ? pending_post_hover_state_
            : GridState::SnakeStopAtCenter;  // safe default
        pending_post_hover_state_ = GridState::Idle;
        pending_post_hover_marker_id_ = -1;
        pending_post_hover_yaw_target_rad_ = yaw_align_target_rad_;
        clearMarkerHover();
        last_recorded_marker_id_ = -1;
        if (next == GridState::SnakeStopAtCenter) {
            snake_stop_stable_count_ = 0;
        }
        transition(next, in.now_s, "marker_hover_done");
    }
}

void GridMission::handleSnakeComplete(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::HoldPosition;
    out.target_yaw_rad = yaw_align_target_rad_;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.advance_phase = false;
    if (completion_hover_marker_id_ >= 0) {
        populateMarkerInputs(in, completion_hover_marker_id_, out);
        out.intent = control::GridControlIntent::MarkerHover;
    }

    // Cycle 24: drain one synthetic completion node per tick. Each drained
    // event is committed to tracker.nodes_ (so the onboard grid model is
    // closed for future Manhattan revisit planning) AND shipped to the GCS
    // via last_committed_event so the map renders as a closed rectangle.
    if (!pending_synth_events_.empty()) {
        out.commit_tracker_advance = true;
        out.synthetic_commit_event = pending_synth_events_.front();
        pending_synth_events_.pop_front();
        // Keep the hover timer paused while we're still draining; reset
        // state_entry_s_ so the post-drain hover_s timer starts after the
        // last synthetic node ships.
        state_entry_s_ = in.now_s;
        return;
    }

    if (in.now_s - state_entry_s_ > config_.snake_complete_hover_s) {
        transition(GridState::RevisitInit, in.now_s, "snake_done_revisit");
    }
}

void GridMission::handleRevisitInit(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::HoldPosition;
    out.target_yaw_rad = yawForHeading(
        tracker_ ? tracker_->currentHeading() : revisit_current_heading_);
    out.target_altitude_m = config_.cruise_altitude_m;
    out.advance_phase = false;
    grid_map_finalized_ = true;

    if (!revisit_route_ready_ && !buildRevisitPlan()) {
        transition(GridState::RevisitComplete, in.now_s, "no_revisit_targets");
        return;
    }
    startCurrentRevisitLeg(in, out);
}

void GridMission::handleRevisitForward(const GridMissionInput& in,
                                       GridMissionOutput& out)
{
    if (!hop_start_local_x_.has_value()) {
        armHopStart(in);
    }
    out.intent = control::GridControlIntent::ForwardBlind;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(currentRevisitSegmentHeading());
    out.advance_phase = false;

    const double distance = hopDistance(in);
    const bool post_turn_blind = in.now_s < snake_post_turn_blind_until_s_;
    if (!post_turn_blind &&
        distance >= config_.hop_intersection_min_distance_m &&
        nodeJustRecorded(in.intersection_decision)) {
        advanceRevisitNode(in);
        out.reason = "revisit_node";
        finishRevisitSegmentOrLeg(in, out);
        return;
    }

    if (distance > config_.hop_max_distance_m ||
        in.now_s - state_entry_s_ > config_.snake_advance_timeout_s) {
        last_safety_event_ = "revisit_hop_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "revisit_hop_timeout");
    }
}

void GridMission::handleRevisitStopAtTurn(const GridMissionInput& in,
                                          GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::StopAndCenter;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(revisit_current_heading_);

    const double vmag = in.local_velocity_xy_mps.value_or(1.0);
    if (vmag <= config_.snake_stop_velocity_threshold_mps) {
        ++snake_stop_stable_count_;
    } else {
        snake_stop_stable_count_ = 0;
    }
    if (snake_stop_stable_count_ < config_.snake_stop_velocity_consecutive_frames) {
        return;
    }
    if (snake_stop_settle_entry_s_ < 0.0) {
        snake_stop_settle_entry_s_ = in.now_s;
    }
    if (in.now_s - snake_stop_settle_entry_s_ < config_.snake_boundary_settle_s) {
        return;
    }

    snake_stop_settle_entry_s_ = -1.0;
    revisit_desired_heading_ = currentRevisitSegmentHeading();
    if (revisit_desired_heading_ == GridHeading::Unknown ||
        revisit_current_heading_ == revisit_desired_heading_) {
        armHopStart(in);
        transition(GridState::RevisitForward, in.now_s, "revisit_no_turn");
        return;
    }
    revisit_turn_step_heading_ =
        nextTurnStepHeading(revisit_current_heading_, revisit_desired_heading_);
    yaw_target_rad_ = yawForHeading(revisit_turn_step_heading_);
    snake_yaw_stable_count_ = 0;
    transition(GridState::RevisitTurn90, in.now_s, "revisit_stopped");
}

void GridMission::handleRevisitTurn90(const GridMissionInput& in,
                                      GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::YawTurn;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yaw_target_rad_;
    if (!in.attitude_yaw_rad.has_value()) {
        return;
    }

    const double err = std::abs(wrap(yaw_target_rad_ - *in.attitude_yaw_rad));
    if (err <= config_.snake_yaw_target_tolerance_rad) {
        ++snake_yaw_stable_count_;
    } else {
        snake_yaw_stable_count_ = 0;
    }
    if (snake_yaw_stable_count_ < config_.snake_yaw_stable_frames) {
        return;
    }

    revisit_current_heading_ = revisit_turn_step_heading_;
    if (tracker_) {
        tracker_->setCurrentPose(revisit_current_coord_, revisit_current_heading_);
    }
    last_turn_complete_s_ = in.now_s;
    snake_yaw_stable_count_ = 0;

    if (revisit_current_heading_ != revisit_desired_heading_) {
        revisit_turn_step_heading_ =
            nextTurnStepHeading(revisit_current_heading_, revisit_desired_heading_);
        yaw_target_rad_ = yawForHeading(revisit_turn_step_heading_);
        transition(GridState::RevisitTurn90, in.now_s, "revisit_turn_step");
        return;
    }

    snake_post_turn_blind_until_s_ = in.now_s + config_.snake_post_turn_blind_s;
    armHopStart(in);
    transition(GridState::RevisitForward, in.now_s, "revisit_turn_done");
}

void GridMission::handleRevisitMarkerHover(const GridMissionInput& in,
                                           GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::MarkerHover;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(revisit_current_heading_);
    out.advance_phase = false;

    const int focus_id = revisit_current_marker_id_;
    if (focus_id >= 0) {
        beginMarkerHover(focus_id, in.now_s);
        populateMarkerInputs(in, focus_id, out);
        if (registry_) {
            auto obs = findMarker(in.vision, focus_id);
            const float orientation = obs.has_value() ? obs->orientation_deg : 0.0f;
            registry_->observe(focus_id,
                               revisit_current_coord_,
                               /*grid_coord_valid=*/true,
                               orientation,
                               in.timestamp_ms,
                               in.frame_seq);
        }
    }

    const bool hover_done = focus_id >= 0
        ? markerHoverComplete(out, in.now_s)
        : (in.now_s - state_entry_s_ >= config_.snake_marker_hover_s);
    if (!hover_done) {
        return;
    }

    if (registry_ && focus_id >= 0) {
        registry_->markRevisited(focus_id, in.timestamp_ms);
    }
    clearMarkerHover();
    last_recorded_marker_id_ = -1;
    ++revisit_leg_index_;
    revisit_segment_index_ = 0;
    revisit_cells_remaining_in_segment_ = 0;
    revisit_current_marker_id_ = -1;
    transition(GridState::RevisitInit, in.now_s, "revisit_marker_done");
}

void GridMission::handleRevisitComplete(const GridMissionInput& in,
                                        GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::HoldPosition;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(
        tracker_ ? tracker_->currentHeading() : revisit_current_heading_);
    transition(GridState::ReturnHomeInit, in.now_s, "revisit_done");
}

void GridMission::handleReturnHomeInit(const GridMissionInput& in,
                                       GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::HoldPosition;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(
        tracker_ ? tracker_->currentHeading() : return_current_heading_);
    out.advance_phase = false;
    grid_map_finalized_ = true;
    grid_pose_visible_ = true;

    if (!return_home_route_ready_ && !buildReturnHomePlan()) {
        last_safety_event_ = "return_home_plan_failed";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "return_home_plan_failed");
        return;
    }
    startCurrentReturnLeg(in, out);
}

void GridMission::handleReturnHomeForward(const GridMissionInput& in,
                                          GridMissionOutput& out)
{
    if (!hop_start_local_x_.has_value()) {
        armHopStart(in);
    }
    out.intent = control::GridControlIntent::ForwardBlind;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(currentReturnSegmentHeading());
    out.advance_phase = false;

    const double distance = hopDistance(in);
    const bool post_turn_blind = in.now_s < snake_post_turn_blind_until_s_;
    if (!post_turn_blind &&
        distance >= config_.hop_intersection_min_distance_m &&
        nodeJustRecorded(in.intersection_decision)) {
        advanceReturnNode(in);
        out.reason = "return_node";
        finishReturnSegmentOrLeg(in, out);
        return;
    }

    if (distance > config_.hop_max_distance_m ||
        in.now_s - state_entry_s_ > config_.snake_advance_timeout_s) {
        last_safety_event_ = "return_hop_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "return_hop_timeout");
    }
}

void GridMission::handleReturnHomeStopAtTurn(const GridMissionInput& in,
                                             GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::StopAndCenter;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(return_current_heading_);

    const double vmag = in.local_velocity_xy_mps.value_or(1.0);
    if (vmag <= config_.snake_stop_velocity_threshold_mps) {
        ++snake_stop_stable_count_;
    } else {
        snake_stop_stable_count_ = 0;
    }
    if (snake_stop_stable_count_ < config_.snake_stop_velocity_consecutive_frames) {
        return;
    }
    if (snake_stop_settle_entry_s_ < 0.0) {
        snake_stop_settle_entry_s_ = in.now_s;
    }
    if (in.now_s - snake_stop_settle_entry_s_ < config_.snake_boundary_settle_s) {
        return;
    }

    snake_stop_settle_entry_s_ = -1.0;
    return_desired_heading_ = currentReturnSegmentHeading();
    if (return_desired_heading_ == GridHeading::Unknown ||
        return_current_heading_ == return_desired_heading_) {
        armHopStart(in);
        transition(GridState::ReturnHomeForward, in.now_s, "return_no_turn");
        return;
    }
    return_turn_step_heading_ =
        nextTurnStepHeading(return_current_heading_, return_desired_heading_);
    yaw_target_rad_ = yawForHeading(return_turn_step_heading_);
    snake_yaw_stable_count_ = 0;
    transition(GridState::ReturnHomeTurn90, in.now_s, "return_stopped");
}

void GridMission::handleReturnHomeTurn90(const GridMissionInput& in,
                                         GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::YawTurn;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yaw_target_rad_;
    if (!in.attitude_yaw_rad.has_value()) {
        return;
    }

    const double err = std::abs(wrap(yaw_target_rad_ - *in.attitude_yaw_rad));
    if (err <= config_.snake_yaw_target_tolerance_rad) {
        ++snake_yaw_stable_count_;
    } else {
        snake_yaw_stable_count_ = 0;
    }
    if (snake_yaw_stable_count_ < config_.snake_yaw_stable_frames) {
        return;
    }

    return_current_heading_ = return_turn_step_heading_;
    if (tracker_) {
        tracker_->setCurrentPose(return_current_coord_, return_current_heading_);
    }
    last_turn_complete_s_ = in.now_s;
    snake_yaw_stable_count_ = 0;

    if (return_current_heading_ != return_desired_heading_) {
        return_turn_step_heading_ =
            nextTurnStepHeading(return_current_heading_, return_desired_heading_);
        yaw_target_rad_ = yawForHeading(return_turn_step_heading_);
        transition(GridState::ReturnHomeTurn90, in.now_s, "return_turn_step");
        return;
    }

    snake_post_turn_blind_until_s_ = in.now_s + config_.snake_post_turn_blind_s;
    armHopStart(in);
    transition(GridState::ReturnHomeForward, in.now_s, "return_turn_done");
}

void GridMission::handleReturnHomeAlignOrigin(const GridMissionInput& in,
                                              GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::IntersectionCenter;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(return_current_heading_);
    out.advance_phase = false;

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
        return_current_coord_ = GridCoord{0, 0};
        if (tracker_) {
            tracker_->setCurrentPose(return_current_coord_, return_current_heading_);
        }
        return_turn_step_heading_ = GridHeading::Unknown;
        snake_yaw_stable_count_ = 0;
        transition(GridState::ReturnHomeFaceSouth, in.now_s, "return_origin_centered");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.entry_center_timeout_s) {
        last_safety_event_ = "return_origin_center_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "return_origin_center_timeout");
    }
}

void GridMission::handleReturnHomeFaceSouth(const GridMissionInput& in,
                                            GridMissionOutput& out)
{
    const GridHeading target_heading = GridHeading::South;
    out.intent = control::GridControlIntent::YawTurn;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(target_heading);
    out.advance_phase = false;

    auto leaveGrid = [&]() {
        return_current_coord_ = GridCoord{0, 0};
        return_current_heading_ = target_heading;
        if (tracker_) {
            tracker_->setCurrentPose(return_current_coord_, return_current_heading_);
        }
        grid_pose_visible_ = false;
        vertiport_acquired_ = false;
        clearMarkerHover();
        armHopStart(in);
        transition(GridState::ReturnVertiportForward, in.now_s, "return_face_south_done");
    };

    if (return_current_heading_ == GridHeading::Unknown ||
        return_current_heading_ == target_heading) {
        leaveGrid();
        return;
    }

    if (return_turn_step_heading_ == GridHeading::Unknown ||
        return_turn_step_heading_ == return_current_heading_) {
        return_turn_step_heading_ =
            nextTurnStepHeading(return_current_heading_, target_heading);
        yaw_target_rad_ = yawForHeading(return_turn_step_heading_);
        snake_yaw_stable_count_ = 0;
    }
    out.target_yaw_rad = yaw_target_rad_;

    if (!in.attitude_yaw_rad.has_value()) {
        return;
    }
    const double err = std::abs(wrap(yaw_target_rad_ - *in.attitude_yaw_rad));
    if (err <= config_.snake_yaw_target_tolerance_rad) {
        ++snake_yaw_stable_count_;
    } else {
        snake_yaw_stable_count_ = 0;
    }
    if (snake_yaw_stable_count_ < config_.snake_yaw_stable_frames) {
        return;
    }

    return_current_heading_ = return_turn_step_heading_;
    if (tracker_) {
        tracker_->setCurrentPose(return_current_coord_, return_current_heading_);
    }
    snake_yaw_stable_count_ = 0;
    if (return_current_heading_ == target_heading) {
        leaveGrid();
        return;
    }

    return_turn_step_heading_ =
        nextTurnStepHeading(return_current_heading_, target_heading);
    yaw_target_rad_ = yawForHeading(return_turn_step_heading_);
    transition(GridState::ReturnHomeFaceSouth, in.now_s, "return_face_south_step");
}

void GridMission::handleReturnVertiportForward(const GridMissionInput& in,
                                               GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::ForwardBlind;
    out.forward_speed_override_mps = config_.entry_forward_speed_mps;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(GridHeading::South);
    out.vertiport_verified = active_vertiport_marker_id_ >= 0;
    out.line_detected = false;
    out.line_center_error_norm = 0.0;
    out.line_angle_error_rad = 0.0;
    out.line_confidence = 0.0;
    out.advance_phase = false;

    const int vertiport_id = active_vertiport_marker_id_;
    if (vertiport_id < 0) {
        last_safety_event_ = "return_vertiport_id_missing";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "return_vertiport_id_missing");
        return;
    }

    if (findMarker(in.vision, vertiport_id).has_value()) {
        vertiport_acquired_ = true;
        clearMarkerHover();
        beginMarkerHover(vertiport_id, in.now_s);
        transition(GridState::ReturnVertiportMarkerHover,
                   in.now_s,
                   "return_vertiport_seen");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.entry_forward_timeout_s) {
        last_safety_event_ = "return_vertiport_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "return_vertiport_timeout");
    }
}

void GridMission::handleReturnVertiportMarkerHover(const GridMissionInput& in,
                                                   GridMissionOutput& out)
{
    const int vertiport_id = active_vertiport_marker_id_;
    out.intent = control::GridControlIntent::MarkerHover;
    out.target_altitude_m = config_.cruise_altitude_m;
    out.target_yaw_rad = yawForHeading(GridHeading::South);
    out.vertiport_verified = vertiport_id >= 0;
    out.advance_phase = false;

    if (vertiport_id < 0) {
        last_safety_event_ = "return_vertiport_id_missing";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "return_vertiport_id_missing");
        return;
    }

    beginMarkerHover(vertiport_id, in.now_s);
    populateMarkerInputs(in, vertiport_id, out);
    if (out.marker_detected) {
        vertiport_acquired_ = true;
    }

    if (out.marker_detected && markerHoverComplete(out, in.now_s)) {
        clearMarkerHover();
        transition(GridState::Land, in.now_s, "return_vertiport_done");
        return;
    }

    if (in.now_s - state_entry_s_ > config_.entry_forward_timeout_s) {
        last_safety_event_ = "return_vertiport_hover_timeout";
        out.last_safety_event = last_safety_event_;
        transition(GridState::EmergencyLand, in.now_s, "return_vertiport_hover_timeout");
    }
}

void GridMission::synthesizeRemainingColumnNodes()
{
    pending_synth_events_.clear();
    if (!tracker_) return;
    const auto& nodes = tracker_->nodes();
    if (nodes.empty()) return;

    // Determine the rectangular grid extent from the columns that have
    // already been traversed end-to-end. We trust min_y/max_y across all
    // committed nodes; by the time we hit SnakeComplete the early columns
    // each span the full range, so the global min/max equals the column
    // length we should close out the current column to.
    int min_y_global = INT_MAX;
    int max_y_global = INT_MIN;
    for (const auto& [coord, ev] : nodes) {
        (void)ev;
        min_y_global = std::min(min_y_global, coord.y);
        max_y_global = std::max(max_y_global, coord.y);
    }

    const GridHeading hd = tracker_->currentHeading();
    const GridCoord cur = tracker_->currentCoord();

    // Only N/S handled. By the time SnakeComplete fires from the
    // SnakeRecordNode "all_markers" branch, the drone is mid-column going
    // either north (-y) or south (+y). E/W only happens during
    // SnakeAdvanceOneCell which is not a SnakeComplete source.
    int target_y = cur.y;
    int step = 0;
    if (hd == GridHeading::North) {
        target_y = min_y_global;
        step = -1;
    } else if (hd == GridHeading::South) {
        target_y = max_y_global;
        step = +1;
    } else {
        return;
    }

    GridCoord walker = cur;
    while (walker.y != target_y) {
        walker.y += step;
        // Skip cells we somehow already have (defensive; should not happen
        // since we're past the last committed node by construction).
        if (nodes.count(walker) != 0) continue;
        GridNodeEvent ev;
        ev.valid = true;
        ev.arrival_heading = hd;
        ev.local_coord = walker;
        ev.topology = onboard::vision::IntersectionType::Cross;
        // Mark all four branches present so the rendered grid edges connect
        // to neighbours and the Manhattan planner sees the cell as a node
        // with full connectivity. Real branch topology is unknown without
        // visiting; "fully connected" is the safe default for a closed grid.
        ev.camera_branch_mask = 0x0F;
        ev.grid_branch_mask = 0x0F;
        ev.origin_local_only = true;
        ev.first_node = false;
        ev.updates_current = false;
        pending_synth_events_.push_back(ev);
    }
}

void GridMission::handleLand(const GridMissionInput& in, GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::Land;
    out.request_land_mode = true;
    if (!in.armed) {
        landing_success_ = true;
        mission_complete_ = true;
        out.mission_finished = true;
        transition(GridState::MissionComplete, in.now_s, "disarmed");
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

void GridMission::handleMissionComplete(const GridMissionInput& in,
                                        GridMissionOutput& out)
{
    out.intent = control::GridControlIntent::Idle;
    out.mission_finished = true;
    landing_success_ = true;
    mission_complete_ = true;
    if (in.now_s - state_entry_s_ > 0.5) {
        transition(GridState::Done, in.now_s, "mission_complete_published");
    }
}

} // namespace onboard::mission
