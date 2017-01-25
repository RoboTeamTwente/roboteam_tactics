#pragma once

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

#include <vector>

namespace rtt {


typedef struct {
	roboteam_utils::Vector2 interceptPos;
	double interceptAngle;
} InterceptPose;

class GetBall : public Skill {
public:
    GetBall(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["getBallAtX"] = BBArgumentType::Double;
        params["getBallAtY"] = BBArgumentType::Double;
        params["getBallAtTime"] = BBArgumentType::Double;
        return params;
    }
    
    std::string node_name() { return "GetBall"; }
private:
	InterceptPose GetInterceptPos(double getBallAtX, double getBallAtY, double getBallAtTime);
	// std::vector<roboteam_utils::Vector2> CircleIntersections(roboteam_utils::Vector2 center, double radius, roboteam_utils::Vector2 startLine, roboteam_utils::Vector2 endLine);
	bool IsPointInCircle(roboteam_utils::Vector2 center, double radius, roboteam_utils::Vector2 point);
	int GetSign(double number);
	int whichRobotHasBall();
	int hasBall;
	double acceptableDeviation = 0.4;
	roboteam_utils::Vector2 prevTargetPos;
	double prevTargetAngle;
	roboteam_msgs::RobotCommand prevCommand;
	AvoidRobots avoidRobots;
	// GoToPos goToPos;
	int robotID;
	bool our_team;
};

extern factories::LeafRegisterer<GetBall, Skill> GetBall_registerer;

} // rtt
