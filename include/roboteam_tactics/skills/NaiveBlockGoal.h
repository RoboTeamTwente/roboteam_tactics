#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"

namespace rtt {

class NaiveBlockGoal : public Skill {
public:
	NaiveBlockGoal(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	// void Initialize() override;
	// double cleanAngle(double angle);
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() { return "NaiveBlockGoal"; }
private:
	ros::NodeHandle n;
	ros::Publisher pubNaiveBlockGoal;

    GoToPos goToPos;

};

} // rtt
