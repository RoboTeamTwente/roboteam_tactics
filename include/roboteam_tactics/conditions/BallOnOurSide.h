#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class BallOnOurSide : public Condition {
    public:
    BallOnOurSide(bt::Blackboard::Ptr blackboard = nullptr);
    
    Status Update();

    private:
    int count = 0;
} ;

}
