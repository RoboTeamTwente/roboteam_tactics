#pragma once

#include "roboteam_utils/TeamRobot.h"
#include "roboteam_utils/Vector2.h"
#include "boost/optional.hpp"
#include <map>

namespace rtt {
    
enum class RefState {
    HALT = 0,
    STOP,
    NORMAL_START,
    FORCED_START,
    PREPARE_KICKOFF_US,
    PREPARE_KICKOFF_THEM,
    PREPARE_PENALTY_US,
    PREPARE_PENALTY_THEM,
    DIRECT_FREE_US,
    DIRECT_FREE_THEM,
    INDIRECT_FREE_US,
    INDIRECT_FREE_THEM,
    TIMEOUT_US,
    TIMEOUT_THEM,
    GOAL_US,
    GOAL_THEM,
    BALL_PLACEMENT_US,
    BALL_PLACEMENT_THEM,
    NORMAL_PLAY
};
  
class RefRule {
public:  
    virtual constexpr RefState getState() const = 0;
    
    virtual bool canMove(const TeamRobot&) const;
    virtual bool canMove(const TeamRobot&, const roboteam_utils::Vector2&) const;

    virtual bool canDribble(const TeamRobot&) const;
    virtual bool canDribble(const TeamRobot&, const roboteam_utils::Vector2&) const;
    
    virtual bool canKick(const TeamRobot&) const;
    
    virtual bool canChip(const TeamRobot&) const;
    
    virtual bool canBeOnOpponentSide(const TeamRobot&) const;
    
    virtual double maxSpeed(const TeamRobot&) const;
    
    virtual double minDistanceToBall(const TeamRobot&) const;
};

#define DEFINE_REF_RULE_HEADER(name) \
  private: \
    constexpr name() {} \
    static const RefRule* SINGLETON; \
  public: \
    constexpr RefState getState() const final override; \
    static const RefRule* get();
    
#define DEFINE_REF_RULE_IMPL(name, state) \
    const RefRule* name::SINGLETON = new name(); \
    constexpr RefState name::getState() const { return state; } \
    const RefRule* name::get() { return SINGLETON; }

class NormalPlayRule : public RefRule {
    DEFINE_REF_RULE_HEADER(NormalPlayRule)
};
 
extern const std::map<RefState, const RefRule*> RULES;
  
}