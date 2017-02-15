#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

class WorstTactic : public Tactic {
    public:
    WorstTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    std::string node_name() { return "WorstTactic"; }

    private:
} ;

}

