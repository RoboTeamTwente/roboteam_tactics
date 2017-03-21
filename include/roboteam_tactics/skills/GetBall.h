#pragma once

#include <vector>

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/Draw.h"

namespace rtt {

/**
 * \class GetBall
 * \brief See YAML
 */
/*
 * Descr: Moves to the ball and optionally kicks it.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *   - AimAt:
 *       Type: String
 *       Can be:
 *         - fieldcenter: The center of the field, (0, 0)
 *         - theirgoal: The opponents' goal
 *         - ourgoal: Our goal
 *         - robot: The robot determined by AimAtRobot and AimAtRobotOurTeam
 *       Descr: Determines where to aim after getting the ball
 *    - AimAtRobot:
 *        Type: Int
 *        Used when: AimAt == robot
 *        Descr: The id of an opponent robot to aim at
 *    - AimAtRobotOurTeam:
 *        Type: Int
 *        Used when: AimAt == robot
 *        Descr: The id of one of our robots to aim at
 *    - passOn:
 *        Type: Bool
 *        Descr: When true, the robot will kick the ball after getting it and aiming in the proper direction.
 */
class GetBall : public Skill {
public:
    GetBall(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update() override;
    void Initialize() override;
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["AimAt"] = BBArgumentType::String;
        params["passOn"] = BBArgumentType::Bool;
        params["AimAtRobot"] = BBArgumentType::Int;
        params["AimAtRobotOurTeam"] = BBArgumentType::Int;
        return params;
    }
    
    static std::vector<std::string> valid_options(const std::string& key) {
        return {
            "fieldcenter",
            "theirgoal",
            "ourgoal",
            "robot"
        };
    }
    
    std::string node_name() { return "GetBall"; }

private:
	int whichRobotHasBall();
	void publishStopCommand();
    
	int robotID;
	int hasBall;
	bool our_team;
	double acceptableDeviation = 0.4;
    bool waiting = true;

	GoToPos goToPos;
    Draw drawer;

    int ballCloseFrameCount;
};

} // rtt
