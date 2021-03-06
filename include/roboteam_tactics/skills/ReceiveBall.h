#pragma once

#include <vector>

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/GetBall.h"
//#include "roboteam_tactics/skills/Kick.h"
//#include "roboteam_tactics/conditions/IsRobotClosestToBall.h"

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_tactics/utils/OpportunityFinder.h"
#include <boost/optional.hpp>

namespace rtt {

/**
 * \class ReceiveBall
 * \brief See YAML
 */
/**
 * Descr: Receives a ball at a certain position
 *
 * Params:
 *  - ROBOT_ID: 
 *      Type: Int
 *      Descr: Id of the robot
 * 
 *  - receiveBallAtCurrentPos: 
 *      Type: Bool
 *      Descr: Indicates that ball should be received at the current robot position
 *
 *  - claimedPos: 
 *      Type: Bool
 *      Descr: Indicates that ball should be received at the position that the robot claimed
 *    
 *  - receiveBallAtX:
 *      Type: Double
 *      Descr: X coord of position at which to receive the ball
 *
 *  - receiveBallAtY:
 *      Type: Double
 *      Descr: Y coord of position at which to receive the ball
 *
 *  - acceptableDeviation:
 *      Type: Double
 *      Used when: receiveBallAtCurrentPos = false
 *      Descr: |
 *        Radius of the acceptable deviation of point of reception.
 *        If reception is the current position of the robot, this
 *        does not do anything
 */

struct InterceptPose {
    Vector2 interceptPos;
    double interceptAngle;
} ;

class ReceiveBall : public Skill {
public:
    ReceiveBall(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
    Vector2 computePoint();
    Status Update();
    void Terminate(Status s);
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["getBallAtX"] = BBArgumentType::Double;
        params["getBallAtY"] = BBArgumentType::Double;
        params["getBallAtTime"] = BBArgumentType::Double;
        return params;
    }
    
    std::string node_name() { return "ReceiveBall"; }
private:
    boost::optional<int> whichRobotHasBall();
    void publishStopCommand();
    InterceptPose deduceInterceptPosFromBall(Vector2 ballPos, Vector2 ballVel, Vector2 myPos);
    boost::optional<InterceptPose> deduceInterceptPosFromRobot();
    boost::optional<int> hasBall;
    
    // output target: grsim or serial
    std::string robot_output_target;

    int robotID;
    bool our_team;
    double acceptableDeviation;
    double marginDeviation;

    GoToPos goToPos;
    Draw drawer;
    bool startKicking;
    bool ballIsComing;
    bool hasTerminated;

    time_point startTime;

    OpportunityFinder opportunityFinder;
    time_point prevComputedPoint;
    Vector2 receiveBallAtPos;
    
};

} // rtt
