#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class GoToLeftTactic : public Tactic {
    public:
    GoToLeftTactic(bt::Blackboard::Ptr blackboard = nullptr);

    Status Update();
} ;

}
