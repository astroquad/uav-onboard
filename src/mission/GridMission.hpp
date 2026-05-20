#pragma once

#include "control/GridControlMapper.hpp"
#include "mission/AltitudePolicy.hpp"
#include "mission/GridCoordinateTracker.hpp"
#include "mission/IntersectionDecision.hpp"
#include "mission/MarkerRegistry.hpp"
#include "mission/SnakePlanner.hpp"
#include "vision/VisionTypes.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace onboard::mission {

enum class GridState {
    Idle,
    ArmTakeoff,
    VertiportVerify,
    VertiportYawAlign,
    OffPadForward,
    LineEnter,
    GridOriginLock,
    SnakeForward,
    SnakeRecordNode,
    SnakeStopAtCenter,
    SnakeTurn90,
    SnakeAdvanceOneCell,
    SnakeTurn90Again,
    SnakeComplete,
    Land,
    EmergencyLand,
    Done,
};

const char* gridStateName(GridState state);

struct GridMissionConfig {
    // ID of the vertiport ArUco marker (DICT_4X4_50, center of vertiport texture).
    int vertiport_marker_id = 23;
    int markers_expected = 4;

    double vertiport_altitude_m = 1.3;
    double cruise_altitude_m = 2.0;

    // Verification timers
    double vertiport_verify_timeout_s = 3.0;
    int    vertiport_marker_stable_frames = 3;
    double vertiport_yaw_tolerance_rad = 0.0872665;   // ~5°
    int    vertiport_yaw_stable_frames = 2;

    // Off-pad forward phase
    double off_pad_timeout_s = 6.0;
    double off_pad_line_locked_frames = 3;
    double off_pad_distance_trigger_m = 2.5;

    // Line enter
    double line_enter_timeout_s = 5.0;

    // Snake forward / record / stop / turn
    double cell_size_m = 3.0;
    double snake_record_lockout_s = 3.0;
    double snake_turn_lockout_s = 3.0;
    double snake_marker_hover_s = 3.0;
    double snake_line_lost_warn_s = 4.0;
    double snake_line_lost_emergency_s = 8.0;
    double snake_stop_velocity_threshold_mps = 0.05;
    int    snake_stop_velocity_consecutive_frames = 2;
    double snake_yaw_target_tolerance_rad = 0.0872665;
    int    snake_yaw_stable_frames = 2;
    double snake_advance_timeout_s = 8.0;

    // Failsafe
    int    max_intersections = 50;
    double mission_timeout_s = 600.0;
    double pixhawk_heartbeat_timeout_s = 2.0;
    double altitude_ceiling_m = 3.0;
    double snake_complete_hover_s = 2.0;

    // CLI/SnakePlanner override
    SnakeTurnDir initial_snake_turn = SnakeTurnDir::Unknown;

    // Cycle 9
    int    marker_observation_min_frames = 3;
    double snake_record_dwell_s = 0.3;

    // Cycle 10: after a NodeRecord, ignore boundary watchdog for this many
    // seconds so the last node's branches don't immediately re-trigger.
    double snake_post_record_grace_s = 0.8;

    // Cycle 12: blind forward duration immediately after each 90 turn
    // (SnakeTurn90 / SnakeTurn90Again). Mirrors the off-pad-forward pattern so
    // the camera moves into the new corridor before re-engaging line tracking
    // and we do not chase the old corridor's leftover line.
    double snake_post_turn_blind_s = 2.5;
};

struct GridMissionInput {
    double now_s = 0.0;
    std::int64_t timestamp_ms = 0;
    std::uint32_t frame_seq = 0;

    bool autopilot_ready = false;
    bool armed = false;
    bool guided_mode = false;
    bool heartbeat_recent = true;
    std::optional<double> rangefinder_m;
    std::optional<double> local_x_m;
    std::optional<double> local_y_m;
    std::optional<double> local_altitude_m;
    std::optional<double> local_velocity_xy_mps;
    std::optional<double> attitude_yaw_rad;
    std::optional<int>    optical_flow_quality;

    const onboard::vision::VisionResult* vision = nullptr;
    IntersectionDecision intersection_decision;
    GridNodeEvent node_event;

    bool land_requested = false;
    bool abort_requested = false;
};

struct GridMissionOutput {
    GridState state = GridState::Idle;
    control::GridControlIntent intent = control::GridControlIntent::Idle;

    double target_altitude_m = 1.3;
    bool   altitude_off_pad_confirmed = false;

    // Inputs forwarded to control mapper
    bool   yaw_available = false;
    double current_yaw_rad = 0.0;
    double target_yaw_rad = 0.0;

    bool   line_detected = false;
    double line_center_error_norm = 0.0;
    double line_angle_error_rad = 0.0;
    double line_confidence = 0.0;
    bool   advance_phase = false;

    bool   marker_detected = false;
    double marker_center_error_x_norm = 0.0;
    double marker_center_error_y_norm = 0.0;

    // High-level controls for the composition root
    bool request_arm_takeoff = false;
    double takeoff_altitude_m = 1.3;
    bool request_land_mode = false;
    bool mission_finished = false;
    bool mission_aborted = false;

    // Cycle 9: when true, the composition root should call
    // tracker.commitAdvance(node_event) to actually move tracker coord forward.
    // GridMission sets this only after lockouts + distance gate pass.
    bool commit_tracker_advance = false;

    // Cycle 12: a one-shot synthetic origin event the composition root should
    // forward to the GCS so the start marker (`s`) is rendered at the
    // vertiport position from the moment the grid is locked, rather than
    // waiting for the first commitAdvance event.
    std::optional<GridNodeEvent> origin_publish_event;

    // Cycle 12 B: cy-feedback deceleration support. The control mapper uses
    // these to taper forward velocity during StopAndCenter so the drone
    // settles right over the intersection center.
    double intersection_center_y_norm = 0.0;
    bool   intersection_valid = false;

    // Cycle 13: drone fractional position relative to the last committed node,
    // in grid units. Used by the GCS to draw the drone heading arrow at a
    // sub-cell position so the user can see the drone progress along the line
    // rather than having the arrow only update at intersection commits.
    bool   drone_position_valid = false;
    double drone_cell_progress = 0.0;   // 0.0 (at last node) ~ ~1.0 (next node)
    double drone_grid_offset_x = 0.0;   // fractional offset along grid X
    double drone_grid_offset_y = 0.0;   // fractional offset along grid Y

    // Telemetry / debug
    GridCoord current_coord;
    GridHeading current_heading = GridHeading::Unknown;
    SnakeTurnDir snake_dir = SnakeTurnDir::Unknown;
    bool vertiport_verified = false;
    int  intersections_recorded = 0;
    std::string reason;
    std::string last_safety_event;
};

class GridMission {
public:
    GridMission(GridMissionConfig config,
                GridCoordinateTracker* tracker,
                MarkerRegistry* registry,
                AltitudePolicy* altitude_policy,
                SnakePlanner* snake_planner,
                IntersectionDecisionEngine* decision_engine = nullptr,
                double node_record_y_min_default = 0.40,
                double node_record_y_max_default = 0.65,
                double node_record_y_min_line_enter = 0.30);

    void start(double now_s);
    GridMissionOutput update(const GridMissionInput& input);
    void reset();

    GridState state() const { return state_; }

    // Cycle 10: how many marker ids currently have a stable-frame count
    // >= marker_observation_min_frames (i.e. eligible to be committed).
    std::size_t stableMarkerCandidateCount() const;

private:
    void transition(GridState next, double now_s, const std::string& reason);

    // State handlers
    void handleArmTakeoff(const GridMissionInput& in, GridMissionOutput& out);
    void handleVertiportVerify(const GridMissionInput& in, GridMissionOutput& out);
    void handleVertiportYawAlign(const GridMissionInput& in, GridMissionOutput& out);
    void handleOffPadForward(const GridMissionInput& in, GridMissionOutput& out);
    void handleLineEnter(const GridMissionInput& in, GridMissionOutput& out);
    void handleGridOriginLock(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeForward(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeRecordNode(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeStopAtCenter(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeTurn90(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeAdvanceOneCell(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeTurn90Again(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeComplete(const GridMissionInput& in, GridMissionOutput& out);
    void handleLand(const GridMissionInput& in, GridMissionOutput& out);
    void handleEmergencyLand(const GridMissionInput& in, GridMissionOutput& out);

    // Helpers
    bool isHardFailsafe(const GridMissionInput& in, std::string& reason) const;
    void populateLineInputs(const GridMissionInput& in, GridMissionOutput& out) const;
    void populateMarkerInputs(const GridMissionInput& in,
                              int marker_id_focus,
                              GridMissionOutput& out) const;
    std::optional<onboard::vision::MarkerObservation> findMarker(
        const onboard::vision::VisionResult* vis, int id) const;
    double wrap(double a) const;

    GridMissionConfig config_;
    GridCoordinateTracker* tracker_;
    MarkerRegistry* registry_;
    AltitudePolicy* altitude_;
    SnakePlanner* snake_;
    IntersectionDecisionEngine* decision_engine_ = nullptr;
    double node_record_y_min_default_ = 0.40;
    double node_record_y_max_default_ = 0.65;
    double node_record_y_min_line_enter_ = 0.30;

    GridState state_ = GridState::Idle;
    double state_entry_s_ = 0.0;
    double mission_start_s_ = 0.0;
    double last_node_record_s_ = -1.0;
    double last_turn_complete_s_ = -1.0;
    int    intersections_recorded_ = 0;
    int    consecutive_boundary_failures_ = 0;
    int    vertiport_marker_stable_count_ = 0;
    int    vertiport_yaw_stable_count_ = 0;
    int    snake_stop_stable_count_ = 0;
    int    snake_yaw_stable_count_ = 0;
    int    off_pad_line_stable_count_ = 0;
    bool   origin_latched_ = false;
    double off_pad_origin_x_ = 0.0;
    double off_pad_origin_y_ = 0.0;
    double yaw_target_rad_ = 0.0;
    double yaw_align_target_rad_ = 0.0;
    double turn_origin_x_ = 0.0;
    double turn_origin_y_ = 0.0;
    int    line_lost_since_node_count_ = 0;
    double line_lost_start_s_ = -1.0;
    SnakeTurnDir pending_turn_dir_ = SnakeTurnDir::Unknown;
    GridHeading pending_post_turn_heading_ = GridHeading::Unknown;
    bool   snake_turn_first_done_ = false;
    bool   marker_hover_active_ = false;
    int    last_recorded_marker_id_ = -1;
    bool   started_ = false;
    std::string last_safety_event_;

    // Cycle 8 additions
    std::optional<double> last_node_local_x_;
    std::optional<double> last_node_local_y_;
    double line_enter_last_seen_s_ = -1.0;
    bool   in_line_enter_relax_ = false; // true while NodeRecord band is relaxed for LineEnter

    // Cycle 9 additions
    std::uint8_t last_node_grid_branch_mask_ = 0;
    bool   last_node_front_open_ = false;
    std::unordered_map<int, int> marker_candidate_count_;

    // Cycle 12 additions
    bool   origin_published_ = false;
    double snake_post_turn_blind_until_s_ = -1.0;
};

} // namespace onboard::mission
