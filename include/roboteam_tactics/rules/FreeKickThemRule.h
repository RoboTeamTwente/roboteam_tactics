#pragma once

#include "Rules.h"

namespace rtt {
    
class FreeKickThemRule : RefRule {
public:
    double minDistanceToBall(const TeamRobot&) const override;
    bool canKick(const TeamRobot&) const override;
    virtual bool canScore(const TeamRobot&) const override = 0;
private:
    constexpr FreeKickThemRule() {}
public:
    class Direct;
    class Indirect;
};

class FreeKickThemRule::Direct : public FreeKickThemRule {
private:
    constexpr Direct() {}
    static const RefRule* SINGLETON;
public:
    constexpr RefState getState() const { return RefState::DIRECT_FREE_THEM; }
    static const RefRule* get();
    bool canScore(const TeamRobot&) const override;
};

class FreeKickThemRule::Indirect : public FreeKickThemRule {
private:
    constexpr Indirect() {}
    static const RefRule* SINGLETON;
public:
    constexpr RefState getState() const { return RefState::INDIRECT_FREE_THEM; }
    static const RefRule* get();
    bool canScore(const TeamRobot&) const override;
};

}