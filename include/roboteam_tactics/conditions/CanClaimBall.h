#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class CanClaimBall
 * \brief See YAML
 */
/*
 * Descr: Checks whether a team has the ball
 * Params:
 *   - our_team:
 *       Type: Bool
 *       Descr: Whether to perform the check for our team, or the opponents'
 */
class CanClaimBall : public Condition {
public:
    CanClaimBall(std::string name, bt::Blackboard::Ptr blackboard);
    Status Update() override;
    
    std::string node_name() { return "CanClaimBall"; }
private:
    
};

}