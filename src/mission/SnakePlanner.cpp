#include "mission/SnakePlanner.hpp"

#include <cstdio>

namespace onboard::mission {
namespace {

constexpr std::uint8_t kNorthBit = 1 << 0;
constexpr std::uint8_t kEastBit = 1 << 1;
constexpr std::uint8_t kSouthBit = 1 << 2;
constexpr std::uint8_t kWestBit = 1 << 3;

GridHeading turnRight(GridHeading heading)
{
    switch (heading) {
    case GridHeading::North: return GridHeading::East;
    case GridHeading::East:  return GridHeading::South;
    case GridHeading::South: return GridHeading::West;
    case GridHeading::West:  return GridHeading::North;
    default: return GridHeading::Unknown;
    }
}

GridHeading turnLeft(GridHeading heading)
{
    switch (heading) {
    case GridHeading::North: return GridHeading::West;
    case GridHeading::East:  return GridHeading::North;
    case GridHeading::South: return GridHeading::East;
    case GridHeading::West:  return GridHeading::South;
    default: return GridHeading::Unknown;
    }
}

std::uint8_t headingBit(GridHeading heading)
{
    switch (heading) {
    case GridHeading::North: return kNorthBit;
    case GridHeading::East:  return kEastBit;
    case GridHeading::South: return kSouthBit;
    case GridHeading::West:  return kWestBit;
    default: return 0;
    }
}

SnakeTurnDir flip(SnakeTurnDir dir)
{
    switch (dir) {
    case SnakeTurnDir::Left:  return SnakeTurnDir::Right;
    case SnakeTurnDir::Right: return SnakeTurnDir::Left;
    default: return SnakeTurnDir::Unknown;
    }
}

} // namespace

SnakePlanner::SnakePlanner(SnakePlannerConfig config) : config_(config)
{
    snake_dir_ = config.initial_turn;
}

GridHeading SnakePlanner::applyTurn(GridHeading heading, SnakeTurnDir dir)
{
    if (dir == SnakeTurnDir::Right) return turnRight(heading);
    if (dir == SnakeTurnDir::Left)  return turnLeft(heading);
    return heading;
}

SnakeTurnDir SnakePlanner::pickInitialTurn(const SnakePlannerInput& input) const
{
    if (input.heading == GridHeading::Unknown) {
        return SnakeTurnDir::Right;
    }
    const std::uint8_t right_bit = headingBit(turnRight(input.heading));
    const std::uint8_t left_bit  = headingBit(turnLeft(input.heading));
    const bool right_open = (input.grid_branch_mask & right_bit) != 0;
    const bool left_open  = (input.grid_branch_mask & left_bit) != 0;
    if (right_open && !left_open) return SnakeTurnDir::Right;
    if (!right_open && left_open) return SnakeTurnDir::Left;
    if (right_open && left_open)  return SnakeTurnDir::Right; // arbitrary; could be CLI-overridden
    return SnakeTurnDir::Unknown;
}

SnakePlannerOutput SnakePlanner::planAtBoundary(const SnakePlannerInput& input)
{
    SnakePlannerOutput out;
    const std::uint8_t front_bit = headingBit(input.heading);
    const bool front_open = (input.grid_branch_mask & front_bit) != 0;

    if (!input.at_boundary_decision) {
        // Mid-segment: forward open means keep going; otherwise treat as boundary
        // decision request (callers should set at_boundary_decision when stopped).
        out.action = front_open ? SnakeAction::Continue : SnakeAction::StopAndTurn;
        out.next_heading = input.heading;
        return out;
    }

    if (input.consecutive_boundary_failure) {
        out.action = SnakeAction::Complete;
        out.boundary_terminated = true;
        out.next_heading = input.heading;
        return out;
    }

    SnakeTurnDir chosen = snake_dir_;
    if (chosen == SnakeTurnDir::Unknown) {
        // Latch the very-first turn direction into snake_dir_ so the
        // subsequent notifySecondTurnCompleted() can alternate from a known
        // reference. While snake_dir_ stays Unknown, flip(Unknown)=Unknown and
        // pickInitialTurn re-runs at every boundary, defaulting to Right
        // whenever both branches are open (i.e. no real alternation).
        chosen = pickInitialTurn(input);
        if (chosen != SnakeTurnDir::Unknown) {
            snake_dir_ = chosen;
            std::fprintf(stderr,
                "[snake] initial latch snake_dir_=%s (mask=0x%02X)\n",
                snakeTurnDirName(snake_dir_), input.grid_branch_mask);
        }
    }
    if (chosen == SnakeTurnDir::Unknown) {
        out.action = SnakeAction::Complete;
        out.boundary_terminated = true;
        return out;
    }
    // Alternation-strict boundary decision. If the snake's expected direction
    // (snake_dir_ alternation) is not present in the latched grid_branch_mask,
    // it means either:
    //   (a) we have reached the grid's terminal column (an L-shape boundary
    //       that forces the OPPOSITE of alternation — a contradiction proving
    //       the snake is complete), OR
    //   (b) vision missed the branch this frame (rare).
    // In either case the safe action is SnakeAction::Complete. Silently
    // flipping to the opposite direction here would make the drone backtrack
    // into the prior column whenever a transient missing branch appears.
    const std::uint8_t chosen_bit = headingBit(applyTurn(input.heading, chosen));
    if ((input.grid_branch_mask & chosen_bit) == 0) {
        std::fprintf(stderr,
            "[snake] alternation mismatch at boundary (snake_dir_=%s, "
            "mask=0x%02X, heading_bit=0x%02X) — snake complete\n",
            snakeTurnDirName(chosen), input.grid_branch_mask, chosen_bit);
        out.action = SnakeAction::Complete;
        out.boundary_terminated = true;
        return out;
    }
    // Do NOT reassign snake_dir_ here. It is the snake's alternation reference
    // (left ↔ right across boundaries) and must stay independent of any
    // per-boundary direction choice above; otherwise a single vision glitch at
    // one boundary could flip the alternation pattern permanently.
    out.action = SnakeAction::StopAndTurn;
    out.plan_turn_dir = chosen;
    out.next_heading = applyTurn(input.heading, chosen);
    // Log every boundary decision so we can audit which direction the planner
    // actually picks vs. the alternation reference.
    std::fprintf(stderr,
        "[snake] planAtBoundary heading=%d grid_mask=0x%02X snake_dir_=%s chosen=%s\n",
        static_cast<int>(input.heading), input.grid_branch_mask,
        snakeTurnDirName(snake_dir_), snakeTurnDirName(chosen));
    return out;
}

void SnakePlanner::notifyFirstTurnCompleted()
{
    first_boundary_seen_ = true;
    // next stage handled by GridMission (SNAKE_ADVANCE_ONE_CELL → SNAKE_TURN_90_AGAIN).
}

void SnakePlanner::notifySecondTurnCompleted()
{
    const SnakeTurnDir before = snake_dir_;
    ++boundaries_completed_;
    snake_dir_ = flip(snake_dir_);
    std::fprintf(stderr,
        "[snake] notifySecondTurnCompleted boundaries=%d snake_dir_ %s -> %s\n",
        boundaries_completed_, snakeTurnDirName(before), snakeTurnDirName(snake_dir_));
}

void SnakePlanner::reset()
{
    snake_dir_ = config_.initial_turn;
    boundaries_completed_ = 0;
    first_boundary_seen_ = false;
    std::fprintf(stderr,
        "[snake] reset() — snake_dir_ initial=%s\n",
        snakeTurnDirName(snake_dir_));
}

const char* snakeTurnDirName(SnakeTurnDir dir)
{
    switch (dir) {
    case SnakeTurnDir::Left:  return "left";
    case SnakeTurnDir::Right: return "right";
    default: return "unknown";
    }
}

} // namespace onboard::mission
