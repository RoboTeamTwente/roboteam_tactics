#include "roboteam_tactics/rules/FreeKickThemRule.h"

namespace rtt {
    
double FreeKickThemRule::minDistanceToBall(const TeamRobot& bot) const {
    return bot.our_team ? .7 : 0;
}

bool FreeKickThemRule::canKick(const TeamRobot& bot) const {
    return !bot.our_team;
}

const RefRule* FreeKickThemRule::Direct::SINGLETON = new FreeKickThemRule::Direct();
const RefRule* FreeKickThemRule::Direct::get() { return SINGLETON; }
bool FreeKickThemRule::Direct::canScore(const TeamRobot&) const { return true; }

const RefRule* FreeKickThemRule::Indirect::SINGLETON = new FreeKickThemRule::Indirect();
const RefRule* FreeKickThemRule::Indirect::get() { return SINGLETON; }
bool FreeKickThemRule::Indirect::canScore(const TeamRobot&) const { return true; } // After we get the ball

}