#pragma once

#include "Rules.h"

namespace rtt {
    
/**
 * \class TimeoutRule
 * \brief Base class for the two TIMEOUT states (US and THEM).
 * No one is allowed to move, dribble, kick or chip.
 */
class TimeoutRule : public RefRule {
public:
    bool canMove(const TeamRobot&) const final override;
    bool canMove(const TeamRobot&, const Vector2&) const final override;

    bool canDribble(const TeamRobot&) const final override;
    bool canDribble(const TeamRobot&, const Vector2&) const final override;
    
    bool canKick(const TeamRobot&) const final override;
    
    bool canChip(const TeamRobot&) const final override;
    
    double maxSpeed(const TeamRobot&) const final override;
    
    class Us;
    class Them;
private:
    constexpr TimeoutRule() {}
};
    
/**
 * \class Us
 * \brief RefRule for the TIMEOUT_US RefState.
 * All rules are in TimeoutRule
 */
class TimeoutRule::Us : public TimeoutRule {
public:
    constexpr RefState getState() const { return RefState::TIMEOUT_US; }
    static const RefRule* get();
private:
    constexpr Us() {}
    static const RefRule* SINGLETON;
};
    
/**
 * \class Them
 * \brief RefRule for the TIMEOUT_THEM RefState.
 * All rules are in TimeoutRule
 */
class TimeoutRule::Them : public TimeoutRule {
public:
    constexpr RefState getState() const { return RefState::TIMEOUT_THEM; }
    static const RefRule* get();
private:
    constexpr Them() {}
    static const RefRule* SINGLETON;
};
    
}