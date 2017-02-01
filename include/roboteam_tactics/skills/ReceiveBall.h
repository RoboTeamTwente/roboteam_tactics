#pragma once

#include <vector>

#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

/**
 * Receives a ball at a certain position
 *
 * Global params:
 *  - ROBOT_ID : Int
 *    Id of the robot
 *
 * Params:
 *  - receiveBallAtCurrentPos : Bool
 *    Indicates that ball should be received at the current robot position
 *    
 *  - receiveBallAtX : Double
 *    X coord of position at which to receive the ball
 *
 *  - receiveBallAtY : Double
 *    Y coord of position at which to receive the ball
 *
 *  - acceptableDeviation : Double
 *    Used when: receiveBallAtCurrentPos = false
 *    Radius of the acceptable deviation of point of reception.
 *    If reception is the current position of the robot, this
 *    does not do anything
 *
 *
 */

struct InterceptPose {
	roboteam_utils::Vector2 interceptPos;
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
	InterceptPose deduceInterceptPosFromBall(double receiveBallAtX, double receiveBallAtY);
	InterceptPose deduceInterceptPosFromRobot(double receiveBallAtX, double receiveBallAtY);
	
	int robotID;
	int hasBall;
	bool our_team;
	double acceptableDeviation = 0.4;

	AvoidRobots avoidRobots;

    bool touchedBall = false;
    time_point initialBallContact;
};

} // rtt
