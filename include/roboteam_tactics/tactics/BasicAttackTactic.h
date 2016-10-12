#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class BasicAttackTactic : public Tactic {
    public:
    BasicAttackTactic(bt::Blackboard::Ptr blackboard = nullptr);

    Status Update();
} ;

}
