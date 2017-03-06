#pragma once

#include "roboteam_utils/TeamRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastRef.h"
#include "boost/optional.hpp"
#include <map>

namespace rtt {
    
class RefRule {
public:  
    virtual constexpr RefState getState() const = 0;
    
    virtual bool canMove(const TeamRobot&) const;
    virtual bool canMove(const TeamRobot&, const roboteam_utils::Vector2&) const;

    virtual bool canDribble(const TeamRobot&) const;
    virtual bool canDribble(const TeamRobot&, const roboteam_utils::Vector2&) const;
    
    virtual bool canKick(const TeamRobot&) const;
    
    virtual bool canChip(const TeamRobot&) const;
    
    virtual bool canScore(const TeamRobot&) const;
    
    virtual bool canBeOnOpponentSide(const TeamRobot&) const;
    
    virtual double maxSpeed(const TeamRobot&) const;
    
    virtual double minDistanceToBall(const TeamRobot&) const;
};

#define DEFINE_REF_RULE_HEADER(name, state) \
  private: \
    constexpr name() {} \
    static const RefRule* SINGLETON; \
  public: \
    constexpr RefState getState() const final override { return state; } \
    static const RefRule* get();
    
#define DEFINE_REF_RULE_IMPL(name) \
    const RefRule* name::SINGLETON = new name(); \
    const RefRule* name::get() { return SINGLETON; }

class NormalPlayRule final : public RefRule {
    DEFINE_REF_RULE_HEADER(NormalPlayRule, RefState::NORMAL_PLAY)
};
 
extern const std::map<RefState, const RefRule*> RULES;
const RefRule* getCurrentRuleSet();
  
}