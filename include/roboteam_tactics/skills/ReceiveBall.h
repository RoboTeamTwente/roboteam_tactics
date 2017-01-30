#pragma once

#include "ros/ros.h"

#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"

#include <vector>

namespace rtt {

typedef struct {
	roboteam_utils::Vector2 interceptPos;
	double interceptAngle;
} InterceptPose;

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

extern factories::LeafRegisterer<ReceiveBall, Skill> ReceiveBall_registerer;

} // rtt
