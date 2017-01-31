#pragma once

#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * Aims the robot at a certain point.
 *
 * Global params:
 *  - ROBOT_ID : Int 
 *    Id of the robot
 * 
 * Params:
 * - At : String
 *   Can be:
 *      robot     - Aim at a robot
 *      theirgoal - Aim at their goal 
 *      ourgoal   - Aim at our goal
 *      position  - Aim at a position
 *   What thing to aim at
 *
 * -- IDEA: Maybe group this under "When At = position:" or something?
 * -- Maybe readable more easily
 * - xGoal : Double
 *   Used when: At = position
 *   The x coord of what to aim at.
 *
 * - yGoal : Double
 *   Used when: At = position
 *   The y coord of what to aim at.
 *
 * - AtRobot : Int
 *   Used when: At = robot
 *   The robot at which to aim.
 */

class AimAt : public Skill {
public:
	AimAt(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
    std::string node_name() { return "AimAt"; }
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["At"] = BBArgumentType::String;
        return params;
    }
    
    static std::vector<std::string> valid_options(const std::string& key) {
        if (key == "At") {
            return std::vector<std::string>({"robot", "ourgoal", "theirgoal"});
        }
        return std::vector<std::string>();
    }
private:
	RotateAroundPoint rotateAroundPoint;

} ;

} // rtt
