#include "mission/GridCoordinateTracker.hpp"
#include "mission/IntersectionDecision.hpp"

#include <cassert>
#include <cstddef>
#include <cstdint>

namespace {

using onboard::mission::GridCoordinateTracker;
using onboard::mission::GridHeading;
using onboard::mission::IntersectionAction;
using onboard::mission::IntersectionDecision;
using onboard::mission::IntersectionDecisionEngine;
using onboard::mission::IntersectionDecisionState;
using onboard::vision::BranchDirection;
using onboard::vision::IntersectionDetection;
using onboard::vision::IntersectionType;

onboard::common::IntersectionDecisionConfig makeConfig()
{
    onboard::common::IntersectionDecisionConfig config;
    config.cruise_window_frames = 6;
    config.turn_confirm_frames = 6;
    config.cooldown_frames = 3;
    config.min_cross_branch_frames = 2;
    config.min_t_branch_frames = 2;
    config.min_l_branch_frames = 3;
    config.min_branch_score = 0.72;
    config.candidate_min_frames = 2;
    config.min_prearm_frames = 2;
    config.front_missing_frames = 2;
    config.record_node_once_frames = 4;
    config.node_advance_min_frames = 4;
    config.turn_zone_y_min = 0.42;
    config.turn_zone_y_max = 0.68;
    config.late_zone_y = 0.78;
    return config;
}

void setBranch(
    IntersectionDetection& detection,
    int index,
    BranchDirection direction,
    bool present,
    float score)
{
    detection.branches[static_cast<std::size_t>(index)].direction = direction;
    detection.branches[static_cast<std::size_t>(index)].present = present;
    detection.branches[static_cast<std::size_t>(index)].score = score;
}

IntersectionDetection makeDetection(
    IntersectionType type,
    std::uint8_t mask,
    float y = 360.0f,
    float score = 0.82f)
{
    IntersectionDetection detection;
    detection.valid = true;
    detection.intersection_detected =
        type == IntersectionType::L ||
        type == IntersectionType::T ||
        type == IntersectionType::Cross;
    detection.type = type;
    detection.raw_type = type;
    detection.score = score;
    detection.raw_score = score;
    detection.center_px = {480.0f, y};
    detection.raw_center_px = detection.center_px;
    setBranch(detection, 0, BranchDirection::Front, (mask & 1) != 0, (mask & 1) != 0 ? 0.88f : 0.20f);
    setBranch(detection, 1, BranchDirection::Right, (mask & 2) != 0, (mask & 2) != 0 ? 0.87f : 0.20f);
    setBranch(detection, 2, BranchDirection::Back, (mask & 4) != 0, (mask & 4) != 0 ? 0.86f : 0.20f);
    setBranch(detection, 3, BranchDirection::Left, (mask & 8) != 0, (mask & 8) != 0 ? 0.85f : 0.20f);
    detection.branch_mask = mask;
    detection.branch_count =
        ((mask & 1) != 0) +
        ((mask & 2) != 0) +
        ((mask & 4) != 0) +
        ((mask & 8) != 0);
    return detection;
}

IntersectionDecision update(
    IntersectionDecisionEngine& engine,
    IntersectionDetection detection,
    std::uint32_t frame,
    bool turn_expected = false)
{
    return engine.update(detection, 960, 720, frame, frame * 83, turn_expected);
}

} // namespace

int main()
{
    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::Unknown, 0, 360.0f), 1);
        decision = update(engine, makeDetection(IntersectionType::Straight, 5, 360.0f), 2);
        assert(decision.action == IntersectionAction::ContinueStraight);
        assert(!decision.event_ready);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        IntersectionDecision decision;
        bool saw_event = false;
        for (std::uint32_t frame = 1; frame <= 3; ++frame) {
            decision = update(engine, makeDetection(IntersectionType::Cross, 15, 360.0f), frame);
            saw_event = saw_event || decision.event_ready;
        }
        assert(decision.accepted_type == IntersectionType::Cross);
        assert(saw_event);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::Cross, 15, 360.0f), 1);
        decision = update(engine, makeDetection(IntersectionType::L, 3, 360.0f), 2);
        decision = update(engine, makeDetection(IntersectionType::L, 3, 360.0f), 3);
        assert(decision.accepted_type != IntersectionType::Cross);
        assert(!decision.event_ready);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::T, 11, 360.0f), 1);
        decision = update(engine, makeDetection(IntersectionType::L, 9, 360.0f), 2);
        decision = update(engine, makeDetection(IntersectionType::Unknown, 1, 360.0f), 3);
        decision = update(engine, makeDetection(IntersectionType::T, 11, 360.0f), 4);
        assert(decision.accepted_type == IntersectionType::T);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::L, 3, 360.0f), 1);
        decision = update(engine, makeDetection(IntersectionType::L, 3, 360.0f), 2);
        decision = update(engine, makeDetection(IntersectionType::Cross, 15, 360.0f), 3);
        decision = update(engine, makeDetection(IntersectionType::L, 3, 360.0f), 4);
        assert(decision.accepted_type == IntersectionType::L);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::T, 11, 360.0f), 1);
        decision = update(engine, makeDetection(IntersectionType::T, 11, 360.0f), 2);
        assert(!decision.required_turn);
        assert(decision.action == IntersectionAction::ContinueStraight);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::L, 2, 360.0f), 1);
        decision = update(engine, makeDetection(IntersectionType::L, 2, 360.0f), 2);
        assert(decision.turn_candidate);
        assert(decision.state == IntersectionDecisionState::TurnReady);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::T, 11, 120.0f), 1, true);
        decision = update(engine, makeDetection(IntersectionType::T, 11, 120.0f), 2, true);
        assert(decision.turn_candidate);
        assert(decision.state != IntersectionDecisionState::TurnReady);
        assert(decision.action == IntersectionAction::PrepareTurn);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::T, 11, 620.0f), 1, true);
        decision = update(engine, makeDetection(IntersectionType::T, 11, 620.0f), 2, true);
        assert(decision.overshoot_risk);
        assert(decision.too_late_to_turn);
        assert(decision.state != IntersectionDecisionState::TurnReady);
    }

    {
        const auto config = makeConfig();
        GridCoordinateTracker tracker(config);
        tracker.setHeading(GridHeading::East);

        IntersectionDecision decision;
        decision.event_ready = true;
        decision.accepted_type = IntersectionType::T;
        decision.accepted_branch_mask = 11;

        auto first = tracker.update(decision, 10, 1000);
        assert(first.valid);
        assert(first.first_node);
        assert(first.local_coord.x == 0);
        assert(first.local_coord.y == 0);

        auto duplicate = tracker.update(decision, 12, 1200);
        assert(!duplicate.valid);

        auto second = tracker.update(decision, 16, 1600);
        assert(second.valid);
        assert(second.local_coord.x == 1);
        assert(second.local_coord.y == 0);
        assert(second.grid_branch_mask == 7);
    }

    {
        const auto config = makeConfig();
        GridCoordinateTracker tracker(config);

        IntersectionDecision decision;
        decision.event_ready = true;
        decision.accepted_type = IntersectionType::L;
        decision.accepted_branch_mask = 3; // front + right at the entry node.

        auto first = tracker.update(decision, 10, 1000);
        assert(first.valid);
        assert(first.first_node);
        assert(first.local_coord.x == 0);
        assert(first.local_coord.y == 0);
        assert(first.arrival_heading == GridHeading::North);
        assert(tracker.currentHeading() == GridHeading::East);

        decision.accepted_type = IntersectionType::T;
        decision.accepted_branch_mask = 13; // front + back + left while moving east.
        auto second = tracker.update(decision, 16, 1600);
        assert(second.valid);
        assert(second.local_coord.x == 1);
        assert(second.local_coord.y == 0);
        assert(second.arrival_heading == GridHeading::East);
        assert(tracker.currentHeading() == GridHeading::East);

        decision.accepted_type = IntersectionType::L;
        decision.accepted_branch_mask = 12; // back + left at row end.
        auto third = tracker.update(decision, 22, 2200);
        assert(third.valid);
        assert(third.local_coord.x == 2);
        assert(third.local_coord.y == 0);
        assert(tracker.currentHeading() == GridHeading::North);

        decision.accepted_type = IntersectionType::T;
        decision.accepted_branch_mask = 13;
        auto fourth = tracker.update(decision, 28, 2800);
        assert(fourth.valid);
        assert(fourth.local_coord.x == 2);
        assert(fourth.local_coord.y == -1);
        assert(fourth.arrival_heading == GridHeading::North);
        assert(tracker.currentHeading() == GridHeading::West);
    }

    {
        assert(onboard::mission::rotateCameraBranchMaskToGrid(11, GridHeading::North) == 11);
        assert(onboard::mission::rotateCameraBranchMaskToGrid(11, GridHeading::East) == 7);
        assert(onboard::mission::rotateCameraBranchMaskToGrid(11, GridHeading::South) == 14);
    }

    {
        IntersectionDecisionEngine engine(makeConfig());
        auto decision = update(engine, makeDetection(IntersectionType::Cross, 15, 360.0f), 1);
        decision = update(engine, makeDetection(IntersectionType::Cross, 15, 360.0f), 2);
        assert(decision.event_ready);
        engine.startCooldown();
        decision = update(engine, makeDetection(IntersectionType::Cross, 15, 360.0f), 3);
        assert(decision.cooldown_active);
        assert(decision.state == IntersectionDecisionState::Cooldown);
        assert(!decision.event_ready);
    }

    return 0;
}
