#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class ParamCheck : public Condition {
    public:
    ParamCheck(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    
    Status Update() override;

    static VerificationMap required_params() {
        VerificationMap params;

        // Name of the param to check
        params["signal"] = BBArgumentType::String;
        // The mode in which to compare the available value.
        // Available choices: lt, gt, neq, eq, leq, geq
        params["mode"] = BBArgumentType::String;
        // The value to which to compare the signal.
        // Can be boolean, int, double, or string.
        // Be reminded that inequality operations (leq, geq) only works
        // on numbers. For boolean and string, only eq/neq works.
        params["value"] = BBArgumentType::String;
        // Rules for determining a value:
        // "true": boolean
        // "false": boolean
        // Contains a '.': double
        // All ints: integer
        // Otherwise: string

        return params;
    }
    std::string node_name() override { return "ParamCheck"; }
    private:
    int count = 0;
} ;

}
