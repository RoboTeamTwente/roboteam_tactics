#pragma once

#include "roboteam_utils/TeamRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastRef.h"
#include "boost/optional.hpp"
#include <map>

namespace rtt {

/**
 * \class RefRule
 * \brief Defines rules for a particular ref command.
 */    
class RefRule {
public:  

    /**
     * \brief Returns the RefState this RefRule is for
     */
    virtual RefState getState() const = 0;
    
    /**
     * \brief Checks whether a robot can move at all
     */
    virtual bool canMove(const TeamRobot&) const;
    
    /**
     * \brief Checks whether a robot can move to a certain position
     */
    virtual bool canMove(const TeamRobot&, const Vector2&) const;

    /**
     * \brief Checks whether a robot can dribble at all
     */
    virtual bool canDribble(const TeamRobot&) const;
    
    /**
     * \brief Checks whether a robot can dribble to a certain position
     */
    virtual bool canDribble(const TeamRobot&, const Vector2&) const;
    
    /**
     * \brief Checks whether a robot can kick the ball
     */
    virtual bool canKick(const TeamRobot&) const;
    
    /**
     * \brief Checks whether a robot can chip the ball
     */
    virtual bool canChip(const TeamRobot&) const;
    
    /**
     * \brief Checks whether a robot is allowed to score a goal
     */
    virtual bool canScore(const TeamRobot&) const;
    
    /**
     * \brief Checks whether a robot is allowed to be on its opponents' side of the field
     */
    virtual bool canBeOnOpponentSide(const TeamRobot&) const;
    
    /**
     * \brief Gets the maximum speed for a robot in m/s
     */
    virtual double maxSpeed(const TeamRobot&) const;
    
    /**
     * \brief Gets the minimum distance between a robot and the ball
     */
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

/**
 * \class NormalPlayRule
 * \brief RefRule for the NORMAL_PLAY RefState.
 * Every rule is at default.
 */
class NormalPlayRule final : public RefRule {
    DEFINE_REF_RULE_HEADER(NormalPlayRule, RefState::NORMAL_PLAY)
};

/**
 * \brief The mapping of RefState to their respective RefRules
 */
 
extern const std::map<RefState, const RefRule*> RULES;

/**
 * \brief Gets the RefRule applicable to the current state of the referee
 */
const RefRule* getCurrentRuleSet();
  
}
