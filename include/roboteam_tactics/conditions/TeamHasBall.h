#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

class TeamHasBall : public Condition {
public:
    static VerificationMap required_params() {
        VerificationMap params;
        params["our_team"] = BBArgumentType::Bool;
        return params;
    }
    
    TeamHasBall(std::string name, bt::Blackboard::Ptr blackboard);
    Status Update() override;
    
    std::string node_name() { return "TeamHasBall"; }
private:
    bool our_team;
};

}