#include "mission/IntersectionDecision.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace onboard::mission {
namespace {

constexpr std::uint8_t kFrontBit = 1 << 0;
constexpr std::uint8_t kRightBit = 1 << 1;
constexpr std::uint8_t kBackBit = 1 << 2;
constexpr std::uint8_t kLeftBit = 1 << 3;

int typePriority(vision::IntersectionType type)
{
    switch (type) {
    case vision::IntersectionType::Cross:
        return 5;
    case vision::IntersectionType::T:
        return 4;
    case vision::IntersectionType::L:
        return 3;
    case vision::IntersectionType::Straight:
        return 2;
    case vision::IntersectionType::Unknown:
        return 1;
    case vision::IntersectionType::None:
        return 0;
    }
    return 0;
}

bool isNodeType(vision::IntersectionType type)
{
    return type == vision::IntersectionType::Cross ||
           type == vision::IntersectionType::T ||
           type == vision::IntersectionType::L;
}

bool hasBit(std::uint8_t mask, std::uint8_t bit)
{
    return (mask & bit) != 0;
}

int branchIndex(vision::BranchDirection direction)
{
    switch (direction) {
    case vision::BranchDirection::Front:
        return 0;
    case vision::BranchDirection::Right:
        return 1;
    case vision::BranchDirection::Back:
        return 2;
    case vision::BranchDirection::Left:
        return 3;
    }
    return 0;
}

float maskConfidence(
    const std::array<BranchEvidence, 4>& evidence,
    std::uint8_t mask)
{
    float sum = 0.0f;
    int count = 0;
    for (int index = 0; index < 4; ++index) {
        if ((mask & (1 << index)) == 0) {
            continue;
        }
        sum += evidence[static_cast<std::size_t>(index)].average_score;
        ++count;
    }
    return count > 0 ? sum / count : 0.0f;
}

bool maskMeets(
    const std::array<BranchEvidence, 4>& evidence,
    std::uint8_t mask,
    int min_frames)
{
    for (int index = 0; index < 4; ++index) {
        if ((mask & (1 << index)) == 0) {
            continue;
        }
        if (evidence[static_cast<std::size_t>(index)].present_frames < min_frames) {
            return false;
        }
    }
    return true;
}

bool maskMaxMeets(
    const std::array<BranchEvidence, 4>& evidence,
    std::uint8_t mask,
    double min_score)
{
    for (int index = 0; index < 4; ++index) {
        if ((mask & (1 << index)) == 0) {
            continue;
        }
        if (evidence[static_cast<std::size_t>(index)].max_score < static_cast<float>(min_score)) {
            return false;
        }
    }
    return true;
}

std::uint8_t bestMask(
    const std::array<BranchEvidence, 4>& evidence,
    const std::array<std::uint8_t, 4>& masks,
    int min_frames)
{
    std::uint8_t output = 0;
    float best_score = -std::numeric_limits<float>::infinity();
    for (const auto mask : masks) {
        if (!maskMeets(evidence, mask, min_frames)) {
            continue;
        }
        const float score = maskConfidence(evidence, mask);
        if (score > best_score) {
            best_score = score;
            output = mask;
        }
    }
    return output;
}

} // namespace

IntersectionDecisionEngine::IntersectionDecisionEngine(const common::IntersectionDecisionConfig& config)
    : config_(config)
{
}

IntersectionDecisionSample IntersectionDecisionEngine::makeSample(
    const vision::IntersectionDetection& intersection,
    std::uint32_t frame_seq,
    std::int64_t timestamp_ms) const
{
    IntersectionDecisionSample sample;
    sample.frame_seq = frame_seq;
    sample.timestamp_ms = timestamp_ms;
    sample.type = intersection.type;
    sample.raw_type = intersection.raw_type;
    sample.valid = intersection.valid && intersection.type != vision::IntersectionType::None;
    sample.stable = intersection.stable;
    sample.held = intersection.held;
    sample.score = intersection.score;
    sample.center_px = intersection.center_px;

    for (const auto& branch : intersection.branches) {
        const int index = branchIndex(branch.direction);
        sample.branch_scores[static_cast<std::size_t>(index)] = branch.score;
        sample.branch_present[static_cast<std::size_t>(index)] =
            sample.valid && branch.score >= static_cast<float>(config_.min_branch_score);
        if (sample.branch_present[static_cast<std::size_t>(index)]) {
            sample.branch_mask = static_cast<std::uint8_t>(sample.branch_mask | (1 << index));
        }
    }

    return sample;
}

std::array<BranchEvidence, 4> IntersectionDecisionEngine::computeBranchEvidence() const
{
    std::array<BranchEvidence, 4> evidence {};
    if (samples_.empty()) {
        return evidence;
    }

    for (const auto& sample : samples_) {
        if (!sample.valid) {
            continue;
        }
        for (std::size_t index = 0; index < evidence.size(); ++index) {
            auto& branch = evidence[index];
            const float score = sample.branch_scores[index];
            branch.max_score = std::max(branch.max_score, score);
            branch.sum_score += score;
            if (sample.branch_present[index]) {
                ++branch.present_frames;
            }
        }
    }

    const float denominator = static_cast<float>(std::max<std::size_t>(1, samples_.size()));
    for (auto& branch : evidence) {
        branch.average_score = branch.sum_score / denominator;
    }
    return evidence;
}

IntersectionDecisionEngine::ClassifiedWindow IntersectionDecisionEngine::classifyWindow(
    const std::array<BranchEvidence, 4>& evidence) const
{
    ClassifiedWindow output;
    for (const auto& sample : samples_) {
        if (!sample.valid) {
            continue;
        }
        if (typePriority(sample.type) > typePriority(output.best_observed_type)) {
            output.best_observed_type = sample.type;
        }
    }

    const auto cross_mask = static_cast<std::uint8_t>(kFrontBit | kRightBit | kBackBit | kLeftBit);
    if (maskMeets(evidence, cross_mask, config_.min_cross_branch_frames) &&
        maskMaxMeets(evidence, cross_mask, config_.high_confidence_score)) {
        output.type = vision::IntersectionType::Cross;
        output.branch_mask = cross_mask;
        output.confidence = maskConfidence(evidence, output.branch_mask);
        return output;
    }

    const std::array<std::uint8_t, 4> t_masks {
        static_cast<std::uint8_t>(kFrontBit | kRightBit | kBackBit),
        static_cast<std::uint8_t>(kRightBit | kBackBit | kLeftBit),
        static_cast<std::uint8_t>(kBackBit | kLeftBit | kFrontBit),
        static_cast<std::uint8_t>(kLeftBit | kFrontBit | kRightBit),
    };
    if (const auto mask = bestMask(evidence, t_masks, config_.min_t_branch_frames); mask != 0) {
        output.type = vision::IntersectionType::T;
        output.branch_mask = mask;
        output.confidence = maskConfidence(evidence, output.branch_mask);
        return output;
    }

    const std::array<std::uint8_t, 4> l_masks {
        static_cast<std::uint8_t>(kFrontBit | kRightBit),
        static_cast<std::uint8_t>(kRightBit | kBackBit),
        static_cast<std::uint8_t>(kBackBit | kLeftBit),
        static_cast<std::uint8_t>(kLeftBit | kFrontBit),
    };
    if (const auto mask = bestMask(evidence, l_masks, config_.min_l_branch_frames); mask != 0) {
        output.type = vision::IntersectionType::L;
        output.branch_mask = mask;
        output.confidence = maskConfidence(evidence, output.branch_mask);
        return output;
    }

    const std::array<std::uint8_t, 4> straight_masks {
        static_cast<std::uint8_t>(kFrontBit | kBackBit),
        static_cast<std::uint8_t>(kRightBit | kLeftBit),
        static_cast<std::uint8_t>(0),
        static_cast<std::uint8_t>(0),
    };
    if (const auto mask = bestMask(evidence, straight_masks, config_.candidate_min_frames); mask != 0) {
        output.type = vision::IntersectionType::Straight;
        output.branch_mask = mask;
        output.confidence = maskConfidence(evidence, output.branch_mask);
        return output;
    }

    output.type = output.best_observed_type == vision::IntersectionType::None
        ? vision::IntersectionType::None
        : vision::IntersectionType::Unknown;
    output.confidence = 0.0f;
    return output;
}

bool IntersectionDecisionEngine::frontMissingRecently() const
{
    int missing = 0;
    for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
        if (it->valid && it->branch_present[0]) {
            break;
        }
        ++missing;
        if (missing >= config_.front_missing_frames) {
            return true;
        }
    }
    return false;
}

std::string IntersectionDecisionEngine::approachPhase(float center_y_norm) const
{
    if (center_y_norm < 0.25f) {
        return "far";
    }
    if (center_y_norm < static_cast<float>(config_.turn_zone_y_min)) {
        return "approaching";
    }
    if (center_y_norm <= static_cast<float>(config_.turn_zone_y_max)) {
        return "turn_zone";
    }
    if (center_y_norm < static_cast<float>(config_.late_zone_y)) {
        return "late";
    }
    return "passed";
}

IntersectionDecision IntersectionDecisionEngine::update(
    const vision::IntersectionDetection& intersection,
    int frame_width,
    int frame_height,
    std::uint32_t frame_seq,
    std::int64_t timestamp_ms,
    bool turn_expected)
{
    IntersectionDecision decision;
    if (!config_.enabled) {
        return decision;
    }

    if (record_lockout_remaining_ > 0) {
        --record_lockout_remaining_;
    }

    if (cooldown_remaining_ > 0) {
        --cooldown_remaining_;
        decision.state = IntersectionDecisionState::Cooldown;
        decision.action = IntersectionAction::ContinueStraight;
        decision.cooldown_active = true;
        return decision;
    }

    samples_.push_back(makeSample(intersection, frame_seq, timestamp_ms));
    const std::size_t max_window = static_cast<std::size_t>(
        std::max(1, std::max(config_.cruise_window_frames, config_.turn_confirm_frames)));
    while (samples_.size() > max_window) {
        samples_.pop_front();
    }

    const auto evidence = computeBranchEvidence();
    const auto classified = classifyWindow(evidence);

    decision.branch_evidence = evidence;
    decision.accepted_type = classified.type;
    decision.best_observed_type = classified.best_observed_type;
    decision.accepted_branch_mask = classified.branch_mask;
    decision.confidence = classified.confidence;
    decision.window_frames = static_cast<int>(samples_.size());
    if (samples_.size() >= 2) {
        decision.age_ms = static_cast<int>(samples_.back().timestamp_ms - samples_.front().timestamp_ms);
    }
    decision.center_px = samples_.empty() ? vision::Point2f {} : samples_.back().center_px;
    if (frame_width > 0 && frame_height > 0) {
        decision.center_y_norm = std::clamp(decision.center_px.y / static_cast<float>(frame_height), 0.0f, 1.0f);
    }
    decision.approach_phase = approachPhase(decision.center_y_norm);

    decision.front_available = evidence[0].present_frames >= config_.candidate_min_frames;
    const bool side_available =
        evidence[1].present_frames >= config_.min_prearm_frames ||
        evidence[3].present_frames >= config_.min_prearm_frames;
    const bool front_missing = frontMissingRecently();
    decision.required_turn = side_available && (turn_expected || front_missing);
    decision.turn_candidate = decision.required_turn;

    const bool in_turn_zone =
        decision.center_y_norm >= static_cast<float>(config_.turn_zone_y_min) &&
        decision.center_y_norm <= static_cast<float>(config_.turn_zone_y_max);
    const bool too_late =
        decision.center_y_norm >= static_cast<float>(config_.late_zone_y) ||
        decision.center_y_norm > static_cast<float>(config_.turn_zone_y_max);
    decision.too_late_to_turn =
        decision.required_turn && decision.center_y_norm >= static_cast<float>(config_.late_zone_y);
    decision.overshoot_risk = decision.required_turn && too_late && !in_turn_zone;

    if (isNodeType(decision.accepted_type) && record_lockout_remaining_ == 0) {
        decision.event_ready = true;
        decision.node_recorded = true;
        record_lockout_remaining_ = std::max(1, config_.record_node_once_frames);
    }

    if (decision.required_turn) {
        if (in_turn_zone) {
            decision.state = IntersectionDecisionState::TurnReady;
            decision.action = IntersectionAction::HoldPosition;
        } else {
            decision.state = IntersectionDecisionState::TurnConfirm;
            decision.action = IntersectionAction::PrepareTurn;
        }
    } else if (decision.event_ready) {
        decision.state = IntersectionDecisionState::NodeRecord;
        decision.action = IntersectionAction::ContinueStraight;
    } else if (classified.type != vision::IntersectionType::None &&
               classified.type != vision::IntersectionType::Unknown) {
        decision.state = IntersectionDecisionState::Candidate;
        decision.action = IntersectionAction::ContinueStraight;
    } else {
        decision.state = IntersectionDecisionState::Cruise;
        decision.action = IntersectionAction::ContinueStraight;
    }

    if (decision.overshoot_risk && decision.state == IntersectionDecisionState::TurnReady) {
        decision.state = IntersectionDecisionState::TurnConfirm;
        decision.action = IntersectionAction::PrepareTurn;
    }
    return decision;
}

void IntersectionDecisionEngine::reset()
{
    samples_.clear();
    cooldown_remaining_ = 0;
    record_lockout_remaining_ = 0;
}

void IntersectionDecisionEngine::startCooldown()
{
    samples_.clear();
    cooldown_remaining_ = std::max(0, config_.cooldown_frames);
}

const char* decisionStateName(IntersectionDecisionState state)
{
    switch (state) {
    case IntersectionDecisionState::Cruise:
        return "cruise";
    case IntersectionDecisionState::Candidate:
        return "candidate";
    case IntersectionDecisionState::NodeRecord:
        return "node_record";
    case IntersectionDecisionState::TurnConfirm:
        return "turn_confirm";
    case IntersectionDecisionState::TurnReady:
        return "turn_ready";
    case IntersectionDecisionState::Cooldown:
        return "cooldown";
    }
    return "unknown";
}

const char* decisionActionName(IntersectionAction action)
{
    switch (action) {
    case IntersectionAction::None:
        return "none";
    case IntersectionAction::ContinueStraight:
        return "continue";
    case IntersectionAction::RecordNode:
        return "record_node";
    case IntersectionAction::PrepareTurn:
        return "prepare_turn";
    case IntersectionAction::TurnLeft:
        return "turn_left";
    case IntersectionAction::TurnRight:
        return "turn_right";
    case IntersectionAction::HoldPosition:
        return "hold";
    }
    return "unknown";
}

} // namespace onboard::mission
