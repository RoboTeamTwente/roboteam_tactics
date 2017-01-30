#pragma once

#include <vector>

#include "roboteam_msgs/World.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class GoToPos : public Skill {
public:
	GoToPos(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
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
    
    std::string node_name() { return "GoToPos"; }
private:
	roboteam_msgs::World prevWorld;

} ;

} // rtt
