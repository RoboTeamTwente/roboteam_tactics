#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include <vector>

#include "roboteam_msgs/navsim.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class FollowPath : public Skill {
public:
    FollowPath(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    std::vector<roboteam_msgs::Vector2f> ComputePath(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 goalPos);
    void CallGoToPos(roboteam_msgs::Vector2f point, double wGoal, int robotID);
	Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["xgoal"] = BBArgumentType::Double;
        params["ygoal"] = BBArgumentType::Double;
        params["wgoal"] = BBArgumentType::Double;
        return params;
    }
    
    std::string node_name() { return "FollowPath"; }
private:

    enum FPState { COMPUTE, GOTO, CHECK };

	ros::NodeHandle n;
	ros::Publisher pub;
	ros::ServiceClient client;
	std::vector<roboteam_msgs::Vector2f> points;

	GoToPos* goToPos;
	int robotID;
	FPState state; // 1: compute path, 2: go to a position along the path, 3: check if done, or move to next position
} ;

} // rtt
