#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

class SingleKeeperPlay : public Tactic {
    public:
    SingleKeeperPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    std::string node_name() { return "SingleKeeperPlay"; }

    private:
} ;

} // rtt

