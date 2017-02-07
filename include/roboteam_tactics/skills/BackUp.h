#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Position.h"

namespace rtt {

// Forward declare skill
class Dribble;
    
class BackUp : public Skill {

    public:
    BackUp(std::string name = "", bt::Blackboard::Ptr bb = nullptr);
    Status Update() override;
    
    std::string node_name() override { return "BackUp"; }
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
    
    private:
    roboteam_utils::Position original_pos;
    roboteam_utils::Position target_pos;
    std::shared_ptr<Dribble> dribble;
}; 
    
}
