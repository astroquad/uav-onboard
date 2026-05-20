#include "mission/SnakePlanner.hpp"

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
        chosen = pickInitialTurn(input);
    }
    if (chosen == SnakeTurnDir::Unknown) {
        out.action = SnakeAction::Complete;
        out.boundary_terminated = true;
        return out;
    }
    // Verify the chosen direction has a branch in grid-frame; if not, flip once.
    const std::uint8_t chosen_bit = headingBit(applyTurn(input.heading, chosen));
    if ((input.grid_branch_mask & chosen_bit) == 0) {
        chosen = flip(chosen);
        const std::uint8_t flipped_bit = headingBit(applyTurn(input.heading, chosen));
        if ((input.grid_branch_mask & flipped_bit) == 0) {
            out.action = SnakeAction::Complete;
            out.boundary_terminated = true;
            return out;
        }
    }
    snake_dir_ = chosen;
    out.action = SnakeAction::StopAndTurn;
    out.plan_turn_dir = chosen;
    out.next_heading = applyTurn(input.heading, chosen);
    return out;
}

void SnakePlanner::notifyFirstTurnCompleted()
{
    first_boundary_seen_ = true;
    // next stage handled by GridMission (SNAKE_ADVANCE_ONE_CELL → SNAKE_TURN_90_AGAIN).
}

void SnakePlanner::notifySecondTurnCompleted()
{
    ++boundaries_completed_;
    snake_dir_ = flip(snake_dir_);
}

void SnakePlanner::reset()
{
    snake_dir_ = config_.initial_turn;
    boundaries_completed_ = 0;
    first_boundary_seen_ = false;
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
