#pragma once

#include <vector>

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/utils/OpportunityFinder.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/Vector2.h"
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
 *       Descr: The ID of the robot
 *
 *   - KEEPER_ID:
 *       Type: Int
 *       Descr: The ID of the keeper
 *
 *   - aimAt:
 *       Type: String
 *       Can be:
 *         - fieldcenter: The center of the field, (0, 0)
 *         - theirgoal: The opponents' goal
 *         - ourgoal: Our goal
 *         - robot: The robot determined by aimAtRobot and ourTeam
 *       Descr: Determines where to aim after getting the ball
 *
 *   - aimAtRobot:
 *       Type: Int
 *       Used when: aimAt == robot
 *       Descr: The id of an opponent robot to aim at
 *
 *   - ourTeam:
 *       Type: Bool
 *       Used when: aimAt == robot
 *       Descr: True if the robot we should aim at is from our team
 *
 *   - targetAngle:
 *        Type: Double
 *        Descr: The angle to look at with the ball
 *        Used when: aimAt is not set
 *
 *   - passOn:
 *        Type: Bool
 *        Descr: When true, the robot will kick the ball after getting it and aiming in the proper direction.
 *        Note: Not sure if this feature should remain, or whether we should always use the kick skill
 *
 *   - stayOnSide:
 *       Descr:     Indicates on which side of the field GoToPos should stay.
 *       Type:      String
 *       Can be:
 *             - ourSide:   Have the robot stay at our side.
 *             - theirSide: Have the robot stay at their side.
 *
 *   - maxSpeed:
 *        Type: Double
 *        Descr: For debugging purposes, a maxSpeed parameter can be set here and will be passed on to GoToPos
 *
 *   - enterDefenseAreas:
 *        Type: Bool
 *        Descr: When true allows GoToPos to go into the defense areas of both teams. Is passed on to GoToPos.
 *
 *   - dribblerOff:
 *        Type: Bool
 *        Descr: When true the dribbler wont be used.
 *
 *   - passToBestAttacker:
 *        Type: Bool
 *        Descr: Aims at best attacker
 *
 *   - doNotPlayBackDefense:
 *        Type: Bool
 *        Descr: Used when passToBestAttacker is true. If doNotPlayBackDefense is true the robot can not play to a robot behind him
 *
 *   - doNotPlayBackAttack:
 *        Type: Bool
 *        Descr: Used when passToBestAttacker is true. If doNotPlayBackAttack is true the robot can not play to a robot which is to far from their goal
 */


class GetBall : public Skill {
public:
    GetBall(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update() override;
    void Initialize() override;
    void Terminate(Status s);
    
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
    
    std::string node_name() override { return "GetBall"; }

private:
    void initializeOpportunityFinder();
    boost::optional<int> whichRobotHasBall();
    void publishStopCommand();
    void publishKickCommand(double kickVel);
    bool claimBall();
    void releaseBall();
    void passBall(int id);
    double computePassSpeed(double dist, double v2);
    
    int robotID;
    std::string robot_output_target = "";
    GoToPos goToPos;
    Draw drawer;
    OpportunityFinder opportunityFinder;

    bool choseRobotToPassTo;
    bool dontDribble;
    bool ballClaimedByMe;
    bool hasTerminated;
    double deviation;

    bool chip;
    double passThreshold;
    int ballCloseFrameCount = 0;
    int bestID;
    Vector2 bestPos;

};

} // rtt
