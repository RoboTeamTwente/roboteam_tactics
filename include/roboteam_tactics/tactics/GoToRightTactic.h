#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class GoToRightTactic : public Tactic {
    public:
    GoToRightTactic(bt::Blackboard::Ptr blackboard = nullptr);

    Status Update();
} ;

}
