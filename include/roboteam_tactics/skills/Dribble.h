#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"

namespace rtt {

class Dribble : public Skill {
public:
	Dribble(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
	void stoprobot(int RobotID);
	double computeAngle(Vector2 robotPos, Vector2 faceTowardsPos);

	double cleanAngle(double angle);
    Vector2 worldToRobotFrame(Vector2 requiredv, double rotation);
    Vector2 saveDribbleDeceleration(Vector2 reqspeed);
    
    /*
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["goalx"] = BBArgumentType::Double;
        params["goaly"] = BBArgumentType::Double;
        return params;
    }*/
    
    std::string node_name() { return "Dribble"; }
private:
	bool goal1reached=false;
	ros::NodeHandle n;
	ros::Publisher pubDebugpoints;
	int robotID;
	Vector2 robotvtogoal=Vector2(0.0,0.0);
	RotateAroundPoint rotateAroundPoint;
	Vector2 prevspeed=Vector2(0.0,0.0);
};


} // rtt
