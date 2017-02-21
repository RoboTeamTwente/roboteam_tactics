#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class BallOnTheirSide : public Condition {
    public:
    BallOnTheirSide(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    
    Status Update() override;

    std::string node_name() { return "BallOnTheirSide"; }
    private:
    int count = 0;
} ;

}
