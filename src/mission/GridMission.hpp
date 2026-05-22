#pragma once

#include "control/GridControlMapper.hpp"
#include "mission/AltitudePolicy.hpp"
#include "mission/GridCoordinateTracker.hpp"
#include "mission/IntersectionDecision.hpp"
#include "mission/MarkerRevisitPlanner.hpp"
#include "mission/MarkerRegistry.hpp"
#include "mission/SnakePlanner.hpp"
#include "vision/VisionTypes.hpp"

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace onboard::mission {

// Cycle 16: state machine rewritten for the new arena layout (no
// vertiport->grid line, free-flight entry forward). Removed states:
// VertiportYawAlign (merged into MarkerLockYaw), OffPadForward, GridOriginLock
// (entry forward hands the first intersection to EntryCenterOrigin before it
// becomes (0,0)). Renamed: VertiportVerify -> MarkerLockYaw, LineEnter -> EntryForward.
enum class GridState {
    Idle,
    ArmTakeoff,
    MarkerLockYaw,
    EntryForward,
    EntryCenterOrigin,
    SnakeForward,
    SnakeRecordNode,
    SnakeLaunchAlign,
    SnakeStopAtCenter,
    SnakeTurn90,
    SnakeAdvanceOneCell,
    SnakeTurn90Again,
    // Cycle 26: 3-second marker hover state for turn nodes. Boundary
    // watchdog and SnakeAdvanceOneCell arrival route through this when a
    // marker is detected at the cell, so turn-cell markers get the same
    // 3-second hover that SnakeRecordNode gives to regular-cell markers.
    // After hover, transitions to whichever GridState was latched in
    // pending_post_hover_state_ (SnakeStopAtCenter for first turn,
    // SnakeTurn90Again for second turn).
    TurnNodeMarkerHover,
    SnakeComplete,
    RevisitInit,
    RevisitForward,
    RevisitStopAtTurn,
    RevisitTurn90,
    RevisitMarkerHover,
    RevisitComplete,
    Land,
    EmergencyLand,
    Done,
};

// Cycle 16: sliding window marker detector. We push the ID seen in each
// vision frame (or -1 when no usable marker is visible). When the same ID
// appears `min_count` times within the last `max_frames` frames the marker is
// considered stable and can be committed. If two distinct IDs ever appear in
// the same window the window is flushed (cleared) — the camera is ambiguous
// and we do not want to commit either ID prematurely.
struct MarkerWindow {
    int max_frames = 8;
    int min_count = 4;
    std::deque<int> ids;        // -1 marks "no marker", positive values are real IDs
    void configure(int frames, int count);
    void clear();
    void push(int id);          // -1 when nothing seen this frame
    bool hasMultipleIds() const;
    int  bestStableId() const;  // returns -1 when none reaches min_count
    int  countOf(int id) const;
};

const char* gridStateName(GridState state);

struct GridMissionConfig {
    // ID of the vertiport ArUco marker (DICT_4X4_50, center of vertiport texture).
    int vertiport_marker_id = 23;
    int markers_expected = 4;

    double vertiport_altitude_m = 2.0;
    double cruise_altitude_m = 2.0;

    // MarkerLockYaw (Cycle 16: merged VertiportVerify + VertiportYawAlign)
    double vertiport_verify_timeout_s = 8.0;
    int    vertiport_marker_stable_frames = 3;
    double vertiport_yaw_tolerance_rad = 0.0872665;   // ~5°
    int    vertiport_yaw_stable_frames = 2;
    // Acceptable marker center error (normalized to half-frame) before we
    // declare the drone "centered over the marker" during MarkerLockYaw.
    double marker_lock_center_tol_norm = 0.05;
    // Body-yaw delta applied during MarkerLockYaw to face the grid entry.
    // ArduPilot/NED convention: yaw_rate > 0 is clockwise (right), so a
    // positive delta rotates the drone to the right. -π/2 would rotate it
    // to the left (CCW) — the opposite of what the mission spec asks for.
    double marker_lock_yaw_delta_rad = 1.5707963;    // +π/2 = right 90°

    // EntryForward (Cycle 16: free-flight forward toward the first grid intersection)
    double entry_forward_timeout_s = 25.0;
    double entry_forward_speed_mps = 0.30;
    double entry_blind_clear_distance_m = 1.8;
    double entry_blind_min_s = 0.0;
    int    entry_blind_min_frames = 0;
    double entry_intersection_min_distance_m = 0.5;
    double entry_center_timeout_s = 12.0;
    double entry_center_target_y_norm = 0.55;
    double entry_center_late_y_norm = 0.78;
    double entry_center_x_tolerance_norm = 0.08;
    double entry_center_y_tolerance_norm = 0.06;
    double entry_center_velocity_threshold_mps = 0.08;
    int    entry_center_stable_frames = 3;

    // Snake forward / record / stop / turn
    double cell_size_m = 3.0;
    double snake_record_lockout_s = 3.0;
    double snake_turn_lockout_s = 3.0;
    // Marker hover holds for the full configured dwell while the controller
    // keeps centering the marker under the camera.
    double snake_marker_hover_s = 3.0;
    double marker_hover_min_s = 0.8;
    double marker_hover_center_tolerance_norm = 0.10;
    int    marker_hover_center_stable_frames = 2;
    double snake_stop_velocity_threshold_mps = 0.05;
    int    snake_stop_velocity_consecutive_frames = 2;
    double snake_yaw_target_tolerance_rad = 0.0872665;
    int    snake_yaw_stable_frames = 2;
    double snake_advance_timeout_s = 15.0;
    // Hop-to-hop progress is measured from the last committed node. During
    // forward hops the mission stays in ForwardBlind: yaw remains locked to
    // the mission target while the mapper borrows only lateral vy from the
    // line controller when a straight corridor is confidently visible.
    // Distance failsafe — if we never see the next intersection within this
    // many metres, give up the hop and let the timeout failsafe kick in.
    double hop_max_distance_m = 3.5;
    // Approach-distance gate before the next intersection commit is honoured.
    // Prevents the last node's residual lookahead from immediately retriggering.
    // Cycle 22: bumped 1.0 -> 2.0 to suppress premature SnakeAdvanceOneCell
    // arrivals caused by the previous boundary T still sitting in the wide
    // downward camera's back-half right after a 90° column-transition turn.
    double hop_intersection_min_distance_m = 2.0;
    double snake_launch_align_timeout_s = 3.0;
    int    snake_launch_align_stable_frames = 2;
    double snake_launch_line_min_confidence = 0.35;
    double snake_launch_line_center_tolerance_norm = 0.06;
    double snake_launch_line_angle_tolerance_rad = 0.0523599; // ~3°

    // Failsafe
    int    max_intersections = 50;
    double mission_timeout_s = 600.0;
    double pixhawk_heartbeat_timeout_s = 2.0;
    double altitude_ceiling_m = 3.5;
    double snake_complete_hover_s = 2.0;

    // CLI/SnakePlanner override
    SnakeTurnDir initial_snake_turn = SnakeTurnDir::Unknown;

    // Dwell at non-marker, non-boundary intersections while branches
    // stabilise. Boundary/turn candidate nodes keep a longer dwell below.
    double snake_record_dwell_s = 0.25;
    double snake_boundary_record_dwell_s = 0.5;
    bool   snake_passthrough_regular_nodes = true;
    bool   revisit_passthrough_regular_nodes = true;
    RevisitOrder revisit_order = RevisitOrder::None;

    // Cycle 10: after a NodeRecord, ignore boundary watchdog for this many
    // seconds so the last node's branches don't immediately re-trigger.
    double snake_post_record_grace_s = 0.8;

    // Cycle 21: after the drone's lateral velocity reaches zero at a boundary,
    // hold position for this long while we keep refreshing
    // last_node_grid_branch_mask_ from the live IntersectionDecision. The
    // watchdog-fire frame can be a 1-frame transient (e.g. straight->T flip
    // where the cross momentarily shows the wrong side branch) and locking
    // that transient into the SnakePlanner causes a wrong alternation read.
    // Giving vision an extra observation window lets a stable T mask win.
    double snake_boundary_settle_s = 1.2;

    // Cycle 12: blind forward duration immediately after each 90 turn
    // (SnakeTurn90 / SnakeTurn90Again). The camera moves into the new corridor
    // before re-engaging line tracking and we do not chase the old corridor's
    // leftover line.
    double snake_post_turn_blind_s = 1.5;

    // Cycle 16: sliding-window ArUco stability gate.
    int marker_window_frames = 8;
    int marker_window_min_count = 4;
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
    std::optional<double> forward_speed_override_mps;

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

    // Cycle 23: optional synthetic GridNodeEvent for boundary commits where
    // IntersectionDecision routed directly to TurnReady (skipping NodeRecord)
    // and tracker peek did not fire a valid event. When set together with
    // commit_tracker_advance=true, the composition root must use this event
    // instead of the peek one. Used so the last boundary node of every snake
    // column reaches last_committed_event -> GCS.
    std::optional<GridNodeEvent> synthetic_commit_event;

    // Cycle 12: a one-shot synthetic origin event the composition root should
    // forward to the GCS so the start marker (`s`) is rendered at the
    // vertiport position from the moment the grid is locked, rather than
    // waiting for the first commitAdvance event.
    std::optional<GridNodeEvent> origin_publish_event;

    // Cycle 12 B: cy-feedback deceleration support. The control mapper uses
    // these to taper forward velocity during StopAndCenter so the drone
    // settles right over the intersection center.
    double intersection_center_x_norm = 0.0;
    double intersection_center_y_norm = 0.0;
    bool   intersection_valid = false;
    double hop_distance_m = 0.0;

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
    bool revisit_active = false;
    bool grid_map_finalized = false;
    std::string revisit_order = "none";
    int revisit_target_id = -1;
    int revisit_remaining = 0;
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

    // Cycle 24: marker id latched by MarkerLockYaw. Returns -1 until a
    // stable candidate has actually been observed, so GCS telemetry can
    // distinguish "not yet seen" from a real vertiport detection.
    int activeVertiportMarkerId() const { return active_vertiport_marker_id_; }

private:
    void transition(GridState next, double now_s, const std::string& reason);

    // State handlers
    void handleArmTakeoff(const GridMissionInput& in, GridMissionOutput& out);
    void handleMarkerLockYaw(const GridMissionInput& in, GridMissionOutput& out);
    void handleEntryForward(const GridMissionInput& in, GridMissionOutput& out);
    void handleEntryCenterOrigin(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeForward(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeRecordNode(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeLaunchAlign(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeStopAtCenter(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeTurn90(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeAdvanceOneCell(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeTurn90Again(const GridMissionInput& in, GridMissionOutput& out);
    void handleTurnNodeMarkerHover(const GridMissionInput& in, GridMissionOutput& out);
    void handleSnakeComplete(const GridMissionInput& in, GridMissionOutput& out);
    void handleRevisitInit(const GridMissionInput& in, GridMissionOutput& out);
    void handleRevisitForward(const GridMissionInput& in, GridMissionOutput& out);
    void handleRevisitStopAtTurn(const GridMissionInput& in, GridMissionOutput& out);
    void handleRevisitTurn90(const GridMissionInput& in, GridMissionOutput& out);
    void handleRevisitMarkerHover(const GridMissionInput& in, GridMissionOutput& out);
    void handleRevisitComplete(const GridMissionInput& in, GridMissionOutput& out);
    void handleLand(const GridMissionInput& in, GridMissionOutput& out);
    void handleEmergencyLand(const GridMissionInput& in, GridMissionOutput& out);

    // Hop-to-hop helper: distance from the latched hop_start_* anchor.
    double hopDistance(const GridMissionInput& in) const;
    void   armHopStart(const GridMissionInput& in);

    // Helpers
    bool isHardFailsafe(const GridMissionInput& in, std::string& reason) const;
    void populateLineInputs(const GridMissionInput& in, GridMissionOutput& out) const;
    void populateMarkerInputs(const GridMissionInput& in,
                              int marker_id_focus,
                              GridMissionOutput& out) const;
    void beginMarkerHover(int marker_id, double now_s);
    void clearMarkerHover();
    bool updateMarkerHoverCenterGate(const GridMissionOutput& out);
    bool markerHoverComplete(const GridMissionOutput& out, double now_s);
    bool hasPendingGridMarkerHint(const GridMissionInput& in) const;
    bool shouldPassThroughRegularNode(const GridMissionInput& in) const;
    bool isEntryIntersectionCandidate(const GridMissionInput& in) const;
    bool isIntersectionCenteredForEntry(const GridMissionInput& in) const;
    double intersectionCenterXNorm(const GridMissionInput& in) const;
    bool isRevisitState() const;
    void populateRevisitTelemetry(GridMissionOutput& out) const;
    void resetRevisitPlan();
    bool buildRevisitPlan();
    bool startCurrentRevisitLeg(const GridMissionInput& in, GridMissionOutput& out);
    bool advanceRevisitNode(const GridMissionInput& in);
    bool finishRevisitSegmentOrLeg(const GridMissionInput& in, GridMissionOutput& out);
    GridHeading currentRevisitSegmentHeading() const;
    GridHeading nextTurnStepHeading(GridHeading from, GridHeading to) const;
    double yawForHeading(GridHeading heading) const;
    void latchGridOrigin(const GridMissionInput& in, GridMissionOutput& out);
    std::optional<onboard::vision::MarkerObservation> findMarker(
        const onboard::vision::VisionResult* vis, int id) const;
    // Cycle 24: any-marker variant. Returns the first marker observation in
    // the current frame, or nullopt if none. Used by MarkerLockYaw to detect
    // the vertiport without hardcoding its id.
    std::optional<onboard::vision::MarkerObservation> findAnyMarker(
        const onboard::vision::VisionResult* vis) const;
    // Cycle 24: returns the marker id we treat as the vertiport at runtime.
    // If MarkerLockYaw has already latched the first stable id we use that;
    // otherwise fall back to the configured default so behaviour matches the
    // pre-Cycle-24 hardcoded path until the dynamic id is known.
    int effectiveVertiportMarkerId() const;
    // Cycle 24: when SnakeComplete fires before reaching the column boundary
    // (all markers found mid-column), synthesize the remaining cells of the
    // current column so the grid stored onboard + sent to GCS is closed.
    void synthesizeRemainingColumnNodes();
    // Cycle 25: at the two turn-cell paths (boundary watchdog and column
    // transition arrival) handleSnakeRecordNode is skipped, so a marker on
    // those cells would otherwise be missed. Registers the current stable
    // marker_window_ id against the supplied grid coord if it is a new
    // non-vertiport id. Returns the id that was registered (or already
    // present at this cell), or -1 if there is no marker to hover on. The
    // caller uses the return value to decide whether to route into
    // TurnNodeMarkerHover for a 3-second hover before continuing.
    int tryRegisterCurrentCellMarker(GridCoord coord, const GridMissionInput& in);
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
    // Cycle 24: dynamic vertiport id. The first marker seen during
    // MarkerLockYaw becomes the vertiport, regardless of config default.
    // `active_vertiport_marker_id_` is set the moment a candidate becomes
    // stable; -1 means "not yet detected, fall back to config default".
    int    active_vertiport_marker_id_ = -1;
    int    candidate_vertiport_id_ = -1;
    int    candidate_vertiport_count_ = 0;
    int    entry_center_stable_count_ = 0;
    std::uint32_t entry_forward_start_frame_seq_ = 0;
    int    snake_stop_stable_count_ = 0;
    double snake_stop_settle_entry_s_ = -1.0;
    int    snake_yaw_stable_count_ = 0;
    int    snake_launch_align_stable_count_ = 0;
    bool   origin_latched_ = false;
    double yaw_target_rad_ = 0.0;
    double yaw_align_target_rad_ = 0.0;
    double turn_origin_x_ = 0.0;
    double turn_origin_y_ = 0.0;
    SnakeTurnDir pending_turn_dir_ = SnakeTurnDir::Unknown;
    GridHeading pending_post_turn_heading_ = GridHeading::Unknown;
    // Cycle 26: GridState to enter after TurnNodeMarkerHover finishes its
    // 3-second hover. Set at the boundary watchdog (= SnakeStopAtCenter)
    // or at SnakeAdvanceOneCell arrival (= SnakeTurn90Again).
    GridState pending_post_hover_state_ = GridState::Idle;
    int pending_post_hover_marker_id_ = -1;
    bool   snake_turn_first_done_ = false;
    bool   marker_hover_active_ = false;
    double marker_hover_start_s_ = -1.0;
    int    marker_hover_centered_count_ = 0;
    int    last_recorded_marker_id_ = -1;
    int    completion_hover_marker_id_ = -1;
    bool   started_ = false;
    std::string last_safety_event_;

    std::optional<double> last_node_local_x_;
    std::optional<double> last_node_local_y_;

    std::uint8_t last_node_grid_branch_mask_ = 0;
    bool   last_node_front_open_ = false;

    // Cycle 24: queue of synthetic GridNodeEvents drained one-per-tick during
    // SnakeComplete after an early all-markers-found exit. Each drained event
    // ends up in tracker.nodes_ (Manhattan revisit planning later) AND in
    // last_committed_event (GCS map rendering).
    std::deque<GridNodeEvent> pending_synth_events_;

    // Cycle 12 additions
    bool   origin_published_ = false;
    double snake_post_turn_blind_until_s_ = -1.0;

    // Cycle 16: hop-to-hop anchor for SnakeForward / SnakeAdvanceOneCell.
    // hop_start_local_x_/y_ is set on state entry; hopDistance() measures the
    // LOCAL_NED displacement from this anchor each tick.
    std::optional<double> hop_start_local_x_;
    std::optional<double> hop_start_local_y_;

    // Cycle 16: sliding-window marker stability detector. Replaces the
    // per-id counter (`marker_candidate_count_`) so transient mis-identifications
    // within a window flush the window instead of accumulating.
    MarkerWindow marker_window_;

    MarkerRevisitPlanner revisit_planner_;
    std::vector<RevisitLeg> revisit_legs_;
    std::size_t revisit_leg_index_ = 0;
    std::size_t revisit_segment_index_ = 0;
    int revisit_cells_remaining_in_segment_ = 0;
    GridCoord revisit_current_coord_;
    GridHeading revisit_current_heading_ = GridHeading::Unknown;
    GridHeading revisit_desired_heading_ = GridHeading::Unknown;
    GridHeading revisit_turn_step_heading_ = GridHeading::Unknown;
    GridHeading revisit_yaw_reference_heading_ = GridHeading::Unknown;
    double revisit_yaw_reference_rad_ = 0.0;
    int revisit_current_marker_id_ = -1;
    bool revisit_route_ready_ = false;
    bool grid_map_finalized_ = false;
};

} // namespace onboard::mission
