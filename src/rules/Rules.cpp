#include "roboteam_tactics/rules/Rules.h"

namespace rtt {
    
bool RefRule::canMove(const TeamRobot&) const { return true; }
bool RefRule::canMove(const TeamRobot&, const roboteam_utils::Vector2&) const { return true; }
bool RefRule::canDribble(const TeamRobot&) const { return true; }
bool RefRule::canDribble(const TeamRobot&, const roboteam_utils::Vector2&) const { return true; }
bool RefRule::canKick(const TeamRobot&) const { return true; }
bool RefRule::canScore(const TeamRobot&) const { return true; }
bool RefRule::canChip(const TeamRobot&) const { return true; }
bool RefRule::canBeOnOpponentSide(const TeamRobot&) const { return true; }
double RefRule::maxSpeed(const TeamRobot&) const { return INFINITY; }
double RefRule::minDistanceToBall(const TeamRobot&) const { return 0.0; }

DEFINE_REF_RULE_IMPL(NormalPlayRule)

const std::map<RefState, const RefRule*> RULES = {
    {RefState::NORMAL_PLAY, NormalPlayRule::get()}
};

}