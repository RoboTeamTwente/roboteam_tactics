#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

class NaiveGetBallWithSupport : public Tactic {
    public:
    NaiveGetBallWithSupport(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    std::string node_name() { return "NaiveGetBallWithSupport"; }

    private:
} ;

} // rtt

