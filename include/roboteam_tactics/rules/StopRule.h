#pragma once

#include "Rules.h"

namespace rtt {
    
class StopRule final : public RefRule {
    DEFINE_REF_RULE_HEADER(StopRule, RefState::STOP)
    
    double minDistanceToBall(const TeamRobot&) const override;    
};
    
}