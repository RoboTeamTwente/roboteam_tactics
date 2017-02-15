#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/BallOnOurSide.h"
#include "roboteam_utils/LastWorld.h"

namespace rtt {
    
class IsRefStage : public Condition {


public:
    IsRefStage(std::string name, bt::Blackboard::Ptr blackboard = nullptr);  
    Status Update() override;
    
    // needs stage which is an integer
    
    /*
    static VerificationMap required_params() {
        VerificationMap params;
        return params;
    }
    */
    
   
private:
   
};
    
}
