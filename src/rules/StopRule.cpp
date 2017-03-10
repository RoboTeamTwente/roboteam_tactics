#include "roboteam_tactics/rules/StopRule.h"

namespace rtt {
    
DEFINE_REF_RULE_IMPL(StopRule)
    
double StopRule::minDistanceToBall(const TeamRobot&) const {
    return .5; // 500 mm
}
    
}