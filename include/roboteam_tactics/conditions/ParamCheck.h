#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * 
 * Descr: |
 *     Checks if a signal is set to a certain value.
 *
 *     Does a comparison of the form:
 *     signalValue [comparison operator] blackboardValue
 *
 *     Value interpretation rules:
 *     - value == true -> Bool
 *     - value == false -> Bool
 *     - value contains "." -> Double
 *     - value is only digits -> Int
 *     - otherwise -> String
 *
 *     @Idea Maybe we need an extra param here that specifies
 *     the robot? Since every robot can use this skill
 *     and you don't want to overwrite eachother's signals
 *     (or get stale ones)
 *
 * Params:
 *     - signal:
 *         Type: String
 *         Descr: Name of the signal that needs to be checked.
 *     
 *     - mode:
 *         Type: String
 *         Descr: |
 *             In which mode to check the value against the signal.
 *             Possible modes:
 *             - lt  (less than)
 *             - gt  (greater than)
 *             - eq  (equals)
 *             - neq (not equals)
 *             - leq (less than or equals)
 *             - geq (greater than or equals)
 *
 *     - value:
 *         Type: String
 *         Descr: The value to compare to the value
 *
 */
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
    BBArgumentType deduceType();
} ;

}
