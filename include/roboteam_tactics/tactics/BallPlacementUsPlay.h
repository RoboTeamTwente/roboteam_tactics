#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class BallPlacementUsPlay : public Tactic {
public:
    BallPlacementUsPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize() override;
    Status Update() override;
    
    std::string node_name() override { return "BallPlacementUsPlay"; }

private:
    std::vector<boost::uuids::uuid> tokens;

} ;

} // rtt


