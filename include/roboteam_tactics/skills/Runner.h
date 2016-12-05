#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

class Runner : public Skill {
public:
	Runner(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        return params;
    }

private:

};


} // rtt
