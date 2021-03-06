#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

namespace rtt_bob {

class BestTactic : public Tactic {
    public:
    BestTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    std::string node_name() { return "BestTactic"; }

    private:
} ;

} // rtt_bob

} // rtt
