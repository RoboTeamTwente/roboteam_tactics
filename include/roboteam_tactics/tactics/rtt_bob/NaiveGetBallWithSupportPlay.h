#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

namespace rtt_bob {

class NaiveGetBallWithSupportPlay : public Tactic {
    public:
    NaiveGetBallWithSupportPlay(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    roboteam_msgs::RoleDirective initSupport(int, int, int, int, bool);
    roboteam_msgs::RoleDirective initSpits();

    void Initialize();

    Status Update();

    std::string node_name() { return "NaiveGetBallWithSupportPlay"; }

    private:
} ;

} // rtt_bob

} // rtt

