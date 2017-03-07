#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

Vector2 getDistToDefenseArea(std::string name, Vector2 point, double safetyMargin);
bool isWithinDefenseArea(std::string whichArea, Vector2 point);

/**
 * \class DistanceXToY
 * \brief See YAML
 */
/*
 * Descr: Checks if measured distance from X to Y is more, less, or equal to the given distance.
 * Params:
 *   ROBOT_ID:
 *     Type: Int
 *     Descr: The robot to check for
 *   distance:
 *     Type: Double
 *     Descr: The distance to compare to
 *   mode:
 *     Type: String
 *     Can be:
 *       lt: less than
 *       gt: greater than
 *       eq: equal to
 *       leq: less than or equal to
 *       geq: greater than or equal to
 *     Descr: The comparison to use
 *   X:
 *     Type: String
 *     Can be:
 *       ball: Distance to the ball
 *       our goal: Distance to our goal
 *       their goal: Distance to their goal
 *       center dot: Distance to (0, 0)
 *       closest opponent: Distance to the closest opponent
 *       some integer: Distance to OUR robot with that id
 *       some integer T: As in "5T" or "0T", distance to the OPPONENT robot with that id.
 *     Descr: The location we want to measure the distance FROM
 *   Y:
 *     Type: String
 *     Can be:
 *       ball: Distance to the ball
 *       our goal: Distance to our goal
 *       their goal: Distance to their goal
 *       center dot: Distance to (0, 0)
 *       closest opponent: Distance to the closest opponent
 *       some integer: Distance to OUR robot with that id
 *       some integer T: As in "5T" or "0T", distance to the OPPONENT robot with that id.
 *     Descr: The location we want to measure the distance TO
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
        // "closest opponent"
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
