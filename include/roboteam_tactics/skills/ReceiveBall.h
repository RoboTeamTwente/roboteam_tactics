#pragma once

#include <vector>

#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

// TODO: Not sure who typed this but this is not the right way :s
// typedef struct {
	// roboteam_utils::Vector2 interceptPos;
	// double interceptAngle;
// } InterceptPose;

// This is the right way:
struct InterceptPose {
	roboteam_utils::Vector2 interceptPos;
	double interceptAngle;
} ;
// (I think the style is the only difference, but consistent style is important)

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
};

} // rtt
