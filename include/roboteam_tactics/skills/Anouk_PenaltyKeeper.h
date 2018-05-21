#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class Anouk_PenaltyKeeper : public Tactic {
public:
    Anouk_PenaltyKeeper(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();
    Status Update();
    
    std::string node_name() override { return "Anouk_PenaltyKeeper"; }

private:
    std::vector<boost::uuids::uuid> tokens;
    Vector2 keeperPosition;
} ;

} // rtt

