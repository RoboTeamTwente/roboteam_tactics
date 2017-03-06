#include "roboteam_tactics/rules/Rules.h"
#include "roboteam_tactics/rules/FreeKickThemRule.h"
#include "roboteam_tactics/rules/FreeKickUsRule.h"
#include "roboteam_tactics/rules/HaltRule.h"
#include "roboteam_tactics/rules/StopRule.h"
#include "roboteam_tactics/rules/TimeoutRule.h"

#include "roboteam_utils/LastRef.h"

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
    {RefState::NORMAL_PLAY, NormalPlayRule::get()},
    {RefState::HALT, HaltRule::get()},
    {RefState::STOP, StopRule::get()},
    {RefState::DIRECT_FREE_US, FreeKickUsRule::Direct::get()},
    {RefState::INDIRECT_FREE_US, FreeKickUsRule::Indirect::get()},
    {RefState::DIRECT_FREE_THEM, FreeKickThemRule::Direct::get()},
    {RefState::INDIRECT_FREE_THEM, FreeKickThemRule::Indirect::get()},
    {RefState::TIMEOUT_US, TimeoutRule::Us::get()},
    {RefState::TIMEOUT_THEM, TimeoutRule::Them::get()}
};

const RefRule* getCurrentRuleSet() {
    return RULES.at(static_cast<RefState>(LastRef::get().command.command));
}

}