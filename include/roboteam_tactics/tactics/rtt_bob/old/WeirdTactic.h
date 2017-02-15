#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

class WeirdTactic : public Tactic {
    public:
    WeirdTactic(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    std::string node_name() { return "WeirdTactic"; }

    private:
} ;

}
