#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class BasicDefenseTactic : public Tactic {
    public:
    BasicDefenseTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    Status Update();
    
    std::string node_name() { return "BasicDefenseTactic"; }
} ;

} // rtt

