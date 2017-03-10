#pragma once
#include "Rules.h"

namespace rtt {
    
/**
 * \class HaltRule
 * \brief RefRule for the HALT RefState.
 * No one is allowed to move, dribble, kick or chip.
 */
class HaltRule final : public RefRule {
    DEFINE_REF_RULE_HEADER(HaltRule, RefState::HALT)
    
    bool canMove(const TeamRobot&) const override;
    bool canMove(const TeamRobot&, const Vector2&) const override;

    bool canDribble(const TeamRobot&) const override;
    bool canDribble(const TeamRobot&, const Vector2&) const override;
    
    bool canKick(const TeamRobot&) const override;
    
    bool canChip(const TeamRobot&) const override;
    
    double maxSpeed(const TeamRobot&) const override;
};
    
}