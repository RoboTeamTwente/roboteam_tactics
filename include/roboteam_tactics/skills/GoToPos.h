#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include <vector>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class GoToPos : public Skill {
public:
	GoToPos(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["xGoal"] = BBArgumentType::Double;
        params["yGoal"] = BBArgumentType::Double;
        params["angleGoal"] = BBArgumentType::Double;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["dribbler"] = BBArgumentType::Bool;
        params["endPoint"] = BBArgumentType::Bool;
        return params;
    }
private:
	roboteam_msgs::World prevWorld;
    ros::NodeHandle n;
	ros::Publisher pub;

} ;

} // rtt
