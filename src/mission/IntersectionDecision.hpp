#pragma once

#include "common/VisionConfig.hpp"
#include "vision/VisionTypes.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <string>

namespace onboard::mission {

enum class IntersectionDecisionState {
    Cruise,
    Candidate,
    NodeRecord,
    TurnConfirm,
    TurnReady,
    Cooldown,
};

enum class IntersectionAction {
    None,
    ContinueStraight,
    RecordNode,
    PrepareTurn,
    TurnLeft,
    TurnRight,
    HoldPosition,
};

struct BranchEvidence {
    int present_frames = 0;
    float max_score = 0.0f;
    float sum_score = 0.0f;
    float average_score = 0.0f;
};

struct IntersectionDecisionSample {
    std::uint32_t frame_seq = 0;
    std::int64_t timestamp_ms = 0;
    vision::IntersectionType type = vision::IntersectionType::None;
    vision::IntersectionType raw_type = vision::IntersectionType::None;
    bool valid = false;
    bool stable = false;
    bool held = false;
    float score = 0.0f;
    std::uint8_t branch_mask = 0;
    std::array<float, 4> branch_scores {};
    std::array<bool, 4> branch_present {};
    vision::Point2f center_px;
};

struct IntersectionDecision {
    IntersectionDecisionState state = IntersectionDecisionState::Cruise;
    IntersectionAction action = IntersectionAction::ContinueStraight;
    vision::IntersectionType accepted_type = vision::IntersectionType::None;
    vision::IntersectionType best_observed_type = vision::IntersectionType::None;
    bool event_ready = false;
    bool turn_candidate = false;
    bool required_turn = false;
    bool front_available = false;
    bool node_recorded = false;
    bool cooldown_active = false;
    std::uint8_t accepted_branch_mask = 0;
    int window_frames = 0;
    int age_ms = 0;
    float confidence = 0.0f;
    vision::Point2f center_px;
    float center_y_norm = 0.0f;
    std::string approach_phase = "far";
    bool overshoot_risk = false;
    bool too_late_to_turn = false;
    std::array<BranchEvidence, 4> branch_evidence {};
};

class IntersectionDecisionEngine {
public:
    explicit IntersectionDecisionEngine(const common::IntersectionDecisionConfig& config);

    IntersectionDecision update(
        const vision::IntersectionDetection& intersection,
        int frame_width,
        int frame_height,
        std::uint32_t frame_seq,
        std::int64_t timestamp_ms,
        bool turn_expected);

    void reset();
    void startCooldown();

private:
    struct ClassifiedWindow {
        vision::IntersectionType type = vision::IntersectionType::None;
        vision::IntersectionType best_observed_type = vision::IntersectionType::None;
        std::uint8_t branch_mask = 0;
        float confidence = 0.0f;
    };

    IntersectionDecisionSample makeSample(
        const vision::IntersectionDetection& intersection,
        std::uint32_t frame_seq,
        std::int64_t timestamp_ms) const;
    std::array<BranchEvidence, 4> computeBranchEvidence() const;
    ClassifiedWindow classifyWindow(const std::array<BranchEvidence, 4>& evidence) const;
    bool frontMissingRecently() const;
    std::string approachPhase(float center_y_norm) const;

    common::IntersectionDecisionConfig config_;
    std::deque<IntersectionDecisionSample> samples_;
    int cooldown_remaining_ = 0;
    int record_lockout_remaining_ = 0;
};

const char* decisionStateName(IntersectionDecisionState state);
const char* decisionActionName(IntersectionAction action);

} // namespace onboard::mission
