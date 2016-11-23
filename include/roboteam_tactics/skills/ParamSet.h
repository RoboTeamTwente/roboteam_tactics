#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class ParamSet : public Skill {
public:
	ParamSet(ros::NodeHandle n, std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	Status Update() override;
    
    static VerificationMap required_params() {
        VerificationMap params;

        // Name of the param to set
        params["signal"] = BBArgumentType::String;
        // The value to which to set the signal/param
        params["value"] = BBArgumentType::String;

        return params;
    }
    std::string node_name() override { return "ParamSet"; }

};


} // rtt
