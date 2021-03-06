#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

/**
 * Max distance ball can be from placement point in meters.
 */
double const MAX_DISTANCE_FROM_END_POINT = 0.1;

class BallPlacementUsPlay : public Tactic {
public:
    BallPlacementUsPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize() override;
    Status Update() override;
    
    std::string node_name() override { return "BallPlacementUsPlay"; }

private:
    boost::uuids::uuid token;
    bool failed;
    bool succeeded;

} ;

} // rtt


