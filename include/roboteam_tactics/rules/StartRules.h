#pragma once

#include "Rules.h"

namespace rtt {

/**
 * \class NormalStartRule
 * \brief RefRule for the NORMAL_START RefState.
 * Everything is at defaults.
 */    
class NormalStartRule : public RefRule {
    DEFINE_REF_RULE_HEADER(NormalStartRule, RefState::NORMAL_START)
};

/**
 * \class ForcedStartRule
 * \brief RefRule for the FORCED_START RefState.
 * Everything is at defaults.
 */    
class ForcedStartRule : public RefRule {
    DEFINE_REF_RULE_HEADER(ForcedStartRule, RefState::FORCED_START)
};
    
}