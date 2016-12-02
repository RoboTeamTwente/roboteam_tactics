#pragma once

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class AimAt : public Skill {
public:
	AimAt(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
    std::string node_name() { return "AimAt"; }
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["At"] = BBArgumentType::String;
        return params;
    }
    
    static std::vector<std::string> valid_options(const std::string& key) {
        if (key == "At") {
            return std::vector<std::string>({"robot", "ourgoal", "theirgoal"});
        }
        return std::vector<std::string>();
    }
private:
    ros::NodeHandle n;
	ros::Publisher pub;
	
	RotateAroundPoint rotateAroundPoint;

} ;

} // rtt
