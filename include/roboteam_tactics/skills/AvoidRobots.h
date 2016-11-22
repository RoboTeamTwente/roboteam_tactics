#pragma once

#include <vector>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class AvoidRobots : public Skill {
public:
	AvoidRobots(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    // roboteam_msgs::RobotCommand SimplePositionController(roboteam_utils::Vector2 posError, double rotError);
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["xGoal"] = BBArgumentType::Double;
        params["yGoal"] = BBArgumentType::Double;
        return params;
    }
    std::string node_name() { return "AvoidRobots"; }
    
private:
    ros::NodeHandle n;
	ros::Publisher pub;

    // Control variables
    double maxSpeed = 1.5;
    double attractiveForce = 10.0;
    double attractiveForceWhenClose = 4.0;
    double repulsiveForce = 10.0;

    double xGoal;
    double yGoal;
    double angleGoal;
    uint   robotID;
    bool   dribbler;

};

} // rtt
