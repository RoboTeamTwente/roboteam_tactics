#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

namespace rtt_bob {

class NaiveGetBallWithSupportPlay : public Tactic {
    public:
    NaiveGetBallWithSupportPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    void Initialize();

    Status Update();

    std::string node_name() { return "NaiveGetBallWithSupportPlay"; }

    private:
} ;

} // rtt_bob

} // rtt

