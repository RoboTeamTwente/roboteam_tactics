#pragma once

#include "Rules.h"

namespace rtt {
    
class NormalStartRule : public RefRule {
    DEFINE_REF_RULE_HEADER(NormalStartRule, RefState::NORMAL_START)
};

class ForcedStartRule : public RefRule {
    DEFINE_REF_RULE_HEADER(ForcedStartRule, RefState::FORCED_START)
};
    
}