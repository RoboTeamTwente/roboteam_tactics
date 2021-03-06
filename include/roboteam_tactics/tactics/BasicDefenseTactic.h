#pragma once

#include "unique_id/unique_id.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class BasicDefenseTactic : public Tactic {
public:
    BasicDefenseTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();
    Status Update();
    
    std::string node_name() override { return "BasicDefenseTactic"; }

private:
    std::vector<boost::uuids::uuid> tokens;
} ;

} // rtt

