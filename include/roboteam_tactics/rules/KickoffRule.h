#pragma once

#include "Rules.h"

namespace rtt {
    
/**
 * \class KickoffRule
 * \brief Base class fot the two PREPARE_KICKOFF rules (US and THEM).
 * No one is allowed to be on their opponents' side.
 */
class KickoffRule : public RefRule {
public:
    bool canBeOnOpponentSide(const TeamRobot&) const final override;
    virtual double minDistanceToBall(const TeamRobot&) const override = 0;
    class Us;
    class Them;
private:
    constexpr KickoffRule() {}
};

/**
 * \class Us
 * \brief RefRule for the PREPARE_KICKOFF_US RefState.
 * In addition to the rule in KickoffRule, the opponents must be at least 50 cm away from the ball.
 */
class KickoffRule::Us : public KickoffRule {
public:
    double minDistanceToBall(const TeamRobot&) const override;
    constexpr RefState getState() const { return RefState::PREPARE_KICKOFF_US; }
    static const RefRule* get();
private:
    constexpr Us() {}
    static const RefRule* SINGLETON;
};

/**
 * \class Them
 * \brief RefRule for the PREPARE_KICKOFF_THEM RefState.
 * In addition to the rule in KickoffRule, we must be at least 50 cm away from the ball.
 */
class KickoffRule::Them : public KickoffRule {
public:
    double minDistanceToBall(const TeamRobot&) const override;
    constexpr RefState getState() const { return RefState::PREPARE_KICKOFF_THEM; }
    static const RefRule* get();
private:
    constexpr Them() {}
    static const RefRule* SINGLETON;
};
    
}