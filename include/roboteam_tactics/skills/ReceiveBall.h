#pragma once

#include <vector>

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/GetBall.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_tactics/utils/ComputePassPoint.h"

namespace rtt {

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
	Status Update();
    
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
	int whichRobotHasBall();
	void publishStopCommand();
	InterceptPose deduceInterceptPosFromBall();
	InterceptPose deduceInterceptPosFromRobot();
	
	int robotID;
	int hasBall;
	bool our_team;
	double acceptableDeviation = 1.0;

	GoToPos goToPos;
	GetBall getBall;
	Draw drawer;
	bool ballHasBeenClose = false;

    bool touchedBall = false;
    time_point initialBallContact;

    PassPoint passPoint;
    time_point prevCheck;
    Vector2 receiveBallAtPos;

    bool computedTargetPos;
};

} // rtt
