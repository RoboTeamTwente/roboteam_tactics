#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/skills/RotateAroundPoint.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class DefendGoalarea : public Skill {
public:
	DefendGoalarea(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    void SetOffsets();

    static VerificationMap required_params() {
        VerificationMap params;
        //params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() override { return "DefendGoalarea"; }
private:
	int robotID;
    bt::Blackboard::Ptr rotate_bb;
    bt::Blackboard::Ptr goto_bb;

    ros::NodeHandle n;
    ros::Publisher debug_pub;

    string our_side;
    double radius;
    double offsetlength=0.0;
    double offsetangle=0.0;
    double robotradius=0.1;
    RotateAroundPoint rotateAroundPoint;
    GoToPos goToPos;

};


} // rtt