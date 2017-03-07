#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class TeamHasBall
 * \brief See YAML
 */
/*
 * Descr: Checks whether a team has the ball
 * Params:
 *   our_team:
 *     Type: Bool
 *     Descr: Whether to perform the check for our team, or the opponents'
 */
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