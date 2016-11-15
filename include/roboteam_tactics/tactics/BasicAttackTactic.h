#pragma once

#include <string>

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class BasicAttackTactic : public Tactic {
    public:
    BasicAttackTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    Status Update();
    
    std::string node_name() { return "BasicAttackTactic"; }
} ;

}
