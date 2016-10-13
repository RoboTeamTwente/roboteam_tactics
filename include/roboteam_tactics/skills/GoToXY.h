#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/Aggregator.h"

namespace rtt {

class GoToXY : public Skill {
    public:
    GoToXY(Aggregator& aggregator, bt::Blackboard::Ptr blackboard = nullptr);

    Status Update();
} ;

} // rtt
