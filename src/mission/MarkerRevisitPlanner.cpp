#include "mission/MarkerRevisitPlanner.hpp"

#include <algorithm>
#include <cmath>

namespace onboard::mission {
namespace {

GridHeading headingForX(int dx)
{
    if (dx > 0) return GridHeading::East;
    if (dx < 0) return GridHeading::West;
    return GridHeading::Unknown;
}

GridHeading headingForY(int dy)
{
    if (dy > 0) return GridHeading::South;
    if (dy < 0) return GridHeading::North;
    return GridHeading::Unknown;
}

int headingIndex(GridHeading heading)
{
    switch (heading) {
    case GridHeading::North: return 0;
    case GridHeading::East:  return 1;
    case GridHeading::South: return 2;
    case GridHeading::West:  return 3;
    case GridHeading::Unknown: return -1;
    }
    return -1;
}

int turnCost(GridHeading from, GridHeading to)
{
    const int a = headingIndex(from);
    const int b = headingIndex(to);
    if (a < 0 || b < 0) return 0;
    const int cw = (b - a + 4) % 4;
    const int ccw = (a - b + 4) % 4;
    return std::min(cw, ccw);
}

void appendSegment(std::vector<RevisitSegment>& segments,
                   GridHeading heading,
                   int cells)
{
    if (heading == GridHeading::Unknown || cells <= 0) {
        return;
    }
    segments.push_back(RevisitSegment{heading, cells});
}

std::vector<RevisitSegment> buildCandidate(GridCoord from,
                                           GridCoord to,
                                           bool x_first)
{
    const int dx = to.x - from.x;
    const int dy = to.y - from.y;
    std::vector<RevisitSegment> segments;
    if (x_first) {
        appendSegment(segments, headingForX(dx), std::abs(dx));
        appendSegment(segments, headingForY(dy), std::abs(dy));
    } else {
        appendSegment(segments, headingForY(dy), std::abs(dy));
        appendSegment(segments, headingForX(dx), std::abs(dx));
    }
    return segments;
}

struct CandidateScore {
    std::vector<RevisitSegment> segments;
    int turns = 0;
    bool first_matches = false;
    int first_cells = 0;
};

CandidateScore scoreCandidate(GridHeading start_heading,
                              std::vector<RevisitSegment> segments)
{
    CandidateScore score;
    score.first_matches =
        !segments.empty() && segments.front().heading == start_heading;
    score.first_cells = segments.empty() ? 0 : segments.front().cells;
    GridHeading heading = start_heading;
    for (const auto& segment : segments) {
        score.turns += turnCost(heading, segment.heading);
        heading = segment.heading;
    }
    score.segments = std::move(segments);
    return score;
}

bool betterCandidate(const CandidateScore& lhs, const CandidateScore& rhs)
{
    if (lhs.turns != rhs.turns) {
        return lhs.turns < rhs.turns;
    }
    if (lhs.first_matches != rhs.first_matches) {
        return lhs.first_matches;
    }
    if (lhs.first_cells != rhs.first_cells) {
        return lhs.first_cells > rhs.first_cells;
    }
    return false;
}

} // namespace

std::vector<RevisitLeg> MarkerRevisitPlanner::buildPlan(
    GridCoord start_coord,
    GridHeading start_heading,
    std::vector<MarkerRevisitTarget> targets,
    RevisitOrder order) const
{
    std::vector<RevisitLeg> legs;
    if (order == RevisitOrder::None || targets.empty()) {
        return legs;
    }

    std::sort(targets.begin(), targets.end(),
        [order](const MarkerRevisitTarget& a, const MarkerRevisitTarget& b) {
            return order == RevisitOrder::Asc ? a.id < b.id : a.id > b.id;
        });

    GridCoord current = start_coord;
    GridHeading heading = start_heading;
    for (const auto& target : targets) {
        const auto x_first = scoreCandidate(
            heading, buildCandidate(current, target.coord, true));
        const auto y_first = scoreCandidate(
            heading, buildCandidate(current, target.coord, false));
        const auto& chosen = betterCandidate(y_first, x_first) ? y_first : x_first;

        RevisitLeg leg;
        leg.marker_id = target.id;
        leg.target = target.coord;
        leg.segments = chosen.segments;
        legs.push_back(leg);

        current = target.coord;
        if (!chosen.segments.empty()) {
            heading = chosen.segments.back().heading;
        }
    }
    return legs;
}

const char* revisitOrderName(RevisitOrder order)
{
    switch (order) {
    case RevisitOrder::None: return "none";
    case RevisitOrder::Asc:  return "asc";
    case RevisitOrder::Desc: return "desc";
    }
    return "none";
}

std::optional<RevisitOrder> parseRevisitOrder(const std::string& value)
{
    if (value == "none" || value == "off" || value == "false") {
        return RevisitOrder::None;
    }
    if (value == "asc" || value == "ascending") {
        return RevisitOrder::Asc;
    }
    if (value == "desc" || value == "descending") {
        return RevisitOrder::Desc;
    }
    return std::nullopt;
}

} // namespace onboard::mission
