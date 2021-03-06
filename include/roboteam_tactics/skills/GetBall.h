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
 */

struct PassOption {
    Vector2 pos;
    int id;
    bool chip;
};

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
    void publishKickCommand(double kickVel, bool chip);
    bool claimBall();
    void releaseBall();
    double passBall(int id, Vector2 pos, Vector2 ballPos, bool chip);
    Vector2 computeBallInterception(Vector2 ballPos, Vector2 ballVel, Vector2 myPos);
    double computePassSpeed(double dist, double input, bool imposeTime);
    double computeArrivalTime(Vector2 location, Vector2 botPos, Vector2 botVel);
    double computeArrivalTime(Vector2 location, int id);
    PassOption choosePassOption(int passID, Vector2 passPos, Vector2 ballPos, const roboteam_msgs::World& world, double passThreshold);
    
    int robotID;
    std::string robot_output_target = "";
    GoToPos goToPos;
    Draw drawer;
    OpportunityFinder opportunityFinder;
    time_point startTime;

    bool choseRobotToPassTo;
    bool dontDribble;
    bool ballClaimedByMe;
    bool hasTerminated;
    double deviation;
    double passThreshold;
    int ballCloseFrameCount = 0;
    int bestID;
    Vector2 bestPos;
    float bestScore = 0;
    bool startKicking;
    bool startChipping;
    double globalKickSpeed;
    int genevaState = 3;
    bool useBallSensor = false;

    boost::optional<Vector2> strafingPos;
    bool isStrafing = false;
};

} // rtt
