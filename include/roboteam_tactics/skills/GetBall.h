#pragma once

#include <vector>

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/Parts.h"

#include "roboteam_utils/Draw.h"

namespace rtt {

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
	int whichRobotHasBall();
	void publishStopCommand();
    
	int robotID;
	int hasBall;
	bool our_team;
	double acceptableDeviation = 0.4;
    bool waiting = true;

	GoToPos goToPos;
    Draw drawer;
};

} // rtt
