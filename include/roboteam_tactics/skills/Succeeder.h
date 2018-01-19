#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class Succeeder
 * \brief See YAML
 */
/*
 * Descr: Utility skill which always succeeds.
 */
class Succeeder : public Skill {
public:
    Succeeder(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        return params;
    }

private:

};


} // rtt
