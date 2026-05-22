#pragma once

#include "mission/GridCoordinateTracker.hpp"

#include <optional>
#include <string>
#include <vector>

namespace onboard::mission {

enum class RevisitOrder {
    None,
    Asc,
    Desc,
};

struct MarkerRevisitTarget {
    int id = -1;
    GridCoord coord;
};

struct RevisitSegment {
    GridHeading heading = GridHeading::Unknown;
    int cells = 0;
};

struct RevisitLeg {
    int marker_id = -1;
    GridCoord target;
    std::vector<RevisitSegment> segments;
};

class MarkerRevisitPlanner {
public:
    RevisitLeg buildLeg(GridCoord start_coord,
                        GridHeading start_heading,
                        GridCoord target,
                        int marker_id = -1) const;

    std::vector<RevisitLeg> buildPlan(GridCoord start_coord,
                                      GridHeading start_heading,
                                      std::vector<MarkerRevisitTarget> targets,
                                      RevisitOrder order) const;
};

const char* revisitOrderName(RevisitOrder order);
std::optional<RevisitOrder> parseRevisitOrder(const std::string& value);

} // namespace onboard::mission
