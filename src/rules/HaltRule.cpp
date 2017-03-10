#include "roboteam_tactics/rules/HaltRule.h"

namespace rtt {

DEFINE_REF_RULE_IMPL(HaltRule)
    
bool HaltRule::canMove(const TeamRobot&) const { return false; }
bool HaltRule::canMove(const TeamRobot&, const Vector2&) const { return false; }

bool HaltRule::canDribble(const TeamRobot&) const { return false; }
bool HaltRule::canDribble(const TeamRobot&, const Vector2&) const { return false; }
   
bool HaltRule::canKick(const TeamRobot&) const { return false; }
    
bool HaltRule::canChip(const TeamRobot&) const { return false; }
    
double HaltRule::maxSpeed(const TeamRobot&) const { return 0.0; }

}