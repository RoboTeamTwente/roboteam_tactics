#pragma once

#include "Rules.h"

namespace rtt {
    
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
    
class TimeoutRule::Us : public TimeoutRule {
public:
    constexpr RefState getState() const { return RefState::TIMEOUT_US; }
    static const RefRule* get();
private:
    constexpr Us() {}
    static const RefRule* SINGLETON;
};
    
class TimeoutRule::Them : public TimeoutRule {
public:
    constexpr RefState getState() const { return RefState::TIMEOUT_THEM; }
    static const RefRule* get();
private:
    constexpr Them() {}
    static const RefRule* SINGLETON;
};
    
}