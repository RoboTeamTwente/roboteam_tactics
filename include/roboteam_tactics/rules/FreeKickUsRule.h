#pragma once

#include "Rules.h"

namespace rtt {
    
class FreeKickUsRule : RefRule {
public:
    double minDistanceToBall(const TeamRobot&) const override;
    bool canKick(const TeamRobot&) const override;
    virtual bool canScore(const TeamRobot&) const override = 0;
private:
    constexpr FreeKickUsRule() {}
public:
    class Direct;
    class Indirect;
};

class FreeKickUsRule::Direct : public FreeKickUsRule {
private:
    constexpr Direct() {}
    static const RefRule* SINGLETON;
public:
    constexpr RefState getState() const { return RefState::DIRECT_FREE_US; }
    static const RefRule* get();
    bool canScore(const TeamRobot&) const override;
};

class FreeKickUsRule::Indirect : public FreeKickUsRule {
private:
    constexpr Indirect() {}
    static const RefRule* SINGLETON;
public:
    constexpr RefState getState() const { return RefState::INDIRECT_FREE_US; }
    static const RefRule* get();
    bool canScore(const TeamRobot&) const override;
};

}