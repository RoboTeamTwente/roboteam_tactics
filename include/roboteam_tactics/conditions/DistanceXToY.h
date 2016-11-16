#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * Checks if measured distance from X to Y is more, less,
 * or equal to the given distance.
 */
class DistanceXToY : public Condition {
    public:

    // TODO: Verificationmap should also check the private bb
    // TODO: Make it possible to set string enums
    static VerificationMap required_params() {
        VerificationMap params;
        // The distance to compare to
        params["distance"] = BBArgumentType::Double;
        // "how" to compare the measured distance to the given
        // distance. Choices: lt, gt, eq, leq, geq
        params["mode"] = BBArgumentType::String;
        // First thing to take the position of. Choices:
        // "ball"
        // "me"
        // "our goal"
        // "their goal"
        // "center dot"
        // An integer (robot on our team)
        // An integer followed by a T (robot on their team)
        // TODO: For later: be able to input an arbitrary vector (2, 2)
        params["X"] = BBArgumentType::String;
        // Second thing to take the position of
        // Same as above
        params["Y"] = BBArgumentType::String;
        return params;
    }
    std::string node_name() { return "DistanceXToY"; }

    DistanceXToY(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    
    Status Update() override;
    private:
    int count = 0;
} ;

}
