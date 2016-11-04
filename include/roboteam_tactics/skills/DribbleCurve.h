#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"

namespace rtt {

class DribbleCurve : public Skill {
public:
	DribbleCurve(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
	void stoprobot(int RobotID);
	double computeAngle(roboteam_utils::Vector2 robotPos, roboteam_utils::Vector2 faceTowardsPos);

	double cleanAngle(double angle);
    roboteam_utils::Vector2 worldToRobotFrame(roboteam_utils::Vector2 requiredv, double rotation);
    /*
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["goalx"] = BBArgumentType::Double;
        params["goaly"] = BBArgumentType::Double;
        return params;
    }*/
    
private:
	ros::NodeHandle n;
	ros::Publisher pubDribbleCurve;
	int robotID;
	
	RotateAroundPoint rotateAroundPoint;
};


} // rtt
