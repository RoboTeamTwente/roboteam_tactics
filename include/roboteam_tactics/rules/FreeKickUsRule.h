#pragma once

#include "Rules.h"

namespace rtt {
    
/**
 * \class FreeKickUsRule
 * \brief Base class for the two FREE_US states (DIRECT and INDIRECT).
 * Only we may kick the ball.
 * The opponents must be at least 50 cm away.
 */
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

/**
 * \class Direct
 * \brief RefRule for the DIRECT_FREE_US RefState.
 * In addition to the rules in FreeKickUsRule, we are allowed to score.
 */
class FreeKickUsRule::Direct : public FreeKickUsRule {
private:
    constexpr Direct() {}
    static const RefRule* SINGLETON;
public:
    constexpr RefState getState() const { return RefState::DIRECT_FREE_US; }
    static const RefRule* get();
    bool canScore(const TeamRobot&) const override;
};

/**
 * \class Indirect
 * \brief RefRule for the INDIRECT_FREE_US RefState.
 * In addition to the rules in FreeKickUsRule, no one is allowed to score.
 */
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