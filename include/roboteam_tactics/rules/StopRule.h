#pragma once

#include "Rules.h"

namespace rtt {
    
/**
 * \class StopRule
 * \brief RefRule for the STOP RefState
 * Everyone must be at least 50 cm away from the ball
 */
class StopRule final : public RefRule {
    DEFINE_REF_RULE_HEADER(StopRule, RefState::STOP)
    
    double minDistanceToBall(const TeamRobot&) const override;    
};
    
}