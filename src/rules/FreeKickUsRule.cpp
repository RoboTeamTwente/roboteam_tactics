#include "roboteam_tactics/rules/FreeKickUsRule.h"

namespace rtt {
    
double FreeKickUsRule::minDistanceToBall(const TeamRobot& bot) const {
    return bot.our_team ? 0 : .7;
}

bool FreeKickUsRule::canKick(const TeamRobot& bot) const {
    return bot.our_team && /* TODO: is free kick taker */ bot.id == 1;
}

const RefRule* FreeKickUsRule::Direct::SINGLETON = new FreeKickUsRule::Direct();
const RefRule* FreeKickUsRule::Direct::get() { return SINGLETON; }
bool FreeKickUsRule::Direct::canScore(const TeamRobot&) const { return true; }

const RefRule* FreeKickUsRule::Indirect::SINGLETON = new FreeKickUsRule::Indirect();
const RefRule* FreeKickUsRule::Indirect::get() { return SINGLETON; }
bool FreeKickUsRule::Indirect::canScore(const TeamRobot&) const { return false; }

}