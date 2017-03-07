#pragma once

#include "Rules.h"

namespace rtt {
    
/**
 * \class FreeKickThemRule
 * \brief Base class for the two FREE_THEM rules (DIRECT and INDIRECT)
 * Only the opponents are allowed to kick the ball.
 * We must be at least 50 cm away.
 */    
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

/**
 * \class Direct
 * \brief RefRule for the DIRECT_FREE_THEM RefState.
 * In addition to the rules in FreeKickThemRule, the opponents are allowed to score.
 */
class FreeKickThemRule::Direct : public FreeKickThemRule {
private:
    constexpr Direct() {}
    static const RefRule* SINGLETON;
public:
    constexpr RefState getState() const { return RefState::DIRECT_FREE_THEM; }
    static const RefRule* get();
    bool canScore(const TeamRobot&) const override;
};

/**
 * \class Indirect
 * \brief RefRule for the INDIRECT_FREE_THEM RefState.
 * In addition to the rules in FreeKickThemRule, no one is allowed to score.
 */
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