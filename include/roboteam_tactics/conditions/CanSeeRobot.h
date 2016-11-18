#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class CanSeeRobot : public Condition {
    public:
    CanSeeRobot(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();

} ;

}
