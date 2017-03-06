#include "roboteam_tactics/rules/KickoffRule.h"

namespace rtt {
    
bool KickoffRule::canBeOnOpponentSide(const TeamRobot&) const { return false; }


double KickoffRule::Us::minDistanceToBall(const TeamRobot& bot) const {
    return bot.our_team ? 0 : .5;
}
const RefRule* KickoffRule::Us::SINGLETON = new Us();
const RefRule* KickoffRule::Us::get() { return SINGLETON; }


double KickoffRule::Them::minDistanceToBall(const TeamRobot& bot) const {
    return bot.our_team ? .5 : 0;
}
const RefRule* KickoffRule::Them::SINGLETON = new Them();
const RefRule* KickoffRule::Them::get() { return SINGLETON; }
    
}