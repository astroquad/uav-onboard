#pragma once

#include "mission/GridCoordinateTracker.hpp"

#include <cstdint>
#include <optional>

namespace onboard::mission {

enum class SnakeTurnDir {
    Unknown,
    Left,
    Right,
};

enum class SnakeAction {
    Continue,        // forward branch present, keep line-following
    StopAndTurn,     // forward blocked, turn 90 in plan_turn_dir
    AdvanceOneCell,  // mid-snake transit after first 90° turn
    SecondTurn,      // after one-cell advance, turn the same direction again
    Complete,        // no more boundaries reachable
};

struct SnakePlannerConfig {
    SnakeTurnDir initial_turn = SnakeTurnDir::Unknown; // CLI override; Unknown = auto
};

struct SnakePlannerInput {
    GridHeading heading = GridHeading::Unknown;
    // Grid-frame branch mask (bit 0=N, 1=E, 2=S, 3=W).
    std::uint8_t grid_branch_mask = 0;
    bool at_boundary_decision = false;  // true at SNAKE_STOP_AT_CENTER
    bool consecutive_boundary_failure = false; // both directions had no line
};

struct SnakePlannerOutput {
    SnakeAction action = SnakeAction::Continue;
    SnakeTurnDir plan_turn_dir = SnakeTurnDir::Unknown;
    GridHeading next_heading = GridHeading::Unknown;
    bool boundary_terminated = false;
};

// Decides snake-search turns at grid boundaries: it latches the first turn
// direction, then enforces strict left/right alternation at each subsequent
// boundary, reporting Complete when the expected alternation branch is absent.
class SnakePlanner {
public:
    explicit SnakePlanner(SnakePlannerConfig config);

    SnakePlannerOutput planAtBoundary(const SnakePlannerInput& input);
    void notifyFirstTurnCompleted();   // call after SNAKE_TURN_90 done
    void notifySecondTurnCompleted();  // call after SNAKE_TURN_90_AGAIN done

    SnakeTurnDir currentSnakeDir() const { return snake_dir_; }
    int boundariesCompleted() const { return boundaries_completed_; }
    void reset();

    static GridHeading applyTurn(GridHeading heading, SnakeTurnDir dir);

private:
    SnakeTurnDir pickInitialTurn(const SnakePlannerInput& input) const;

    SnakePlannerConfig config_;
    SnakeTurnDir snake_dir_ = SnakeTurnDir::Unknown;
    int boundaries_completed_ = 0;
    bool first_boundary_seen_ = false;
};

const char* snakeTurnDirName(SnakeTurnDir dir);

} // namespace onboard::mission
