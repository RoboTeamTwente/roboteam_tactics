#pragma once

#import "Rules.h"

namespace rtt {
    
class KickoffRule : public RefRule {
public:
    bool canBeOnOpponentSide(const TeamRobot&) const final override;
    virtual double minDistanceToBall(const TeamRobot&) const override = 0;
    class Us;
    class Them;
private:
    constexpr KickoffRule() {}
};

class KickoffRule::Us : public RefRule {
public:
    double minDistanceToBall(const TeamRobot&) const override;
    constexpr RefState getState() const { return RefState::PREPARE_KICKOFF_THEM; }
    static const RefRule* get();
private:
    constexpr Us() {}
    static const RefRule* SINGLETON;
};

class KickoffRule::Them : public RefRule {
public:
    double minDistanceToBall(const TeamRobot&) const override;
    constexpr RefState getState() const { return RefState::PREPARE_KICKOFF_THEM; }
    static const RefRule* get();
private:
    constexpr Them() {}
    static const RefRule* SINGLETON;
};
    
}