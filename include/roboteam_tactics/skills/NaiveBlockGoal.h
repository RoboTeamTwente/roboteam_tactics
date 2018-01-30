#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_utils/Draw.h"

namespace rtt {

/**
 * \class NaiveBlockGoal
 * \brief See YAML
 */
/**
 * Descr: |
 *   Tries to stay between the ball and the center of the goal. If the ball goes behind the goal
 *   it just follows it's y coordinate. If the ball enters the goal area the keeper tries to
 *   go to the point between the ball and the center of the goal. Howver, in some cases this
 *   leads to the keeper pushing the ball into the goal. Therefore be sure to cover this case
 *   in the behaviour tree!
 *
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 */
class NaiveBlockGoal : public Skill {
public:
	NaiveBlockGoal(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() { return "NaiveBlockGoal"; }

private:
    GoToPos goToPos;
    Draw drawer;

};

} // rtt
