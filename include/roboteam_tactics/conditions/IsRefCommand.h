#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/conditions/BallOnOurSide.h"
#include "roboteam_utils/LastWorld.h"

namespace rtt {
    
/**
 * \class IsRefCommand
 * \brief See YAML
 */
/*
 * Descr: Checks whether a certain ref command is currently active
 * Params:
 *   command:
 *     Type: String
 *     Descr: The ref command to check for. See RefLookup.h for valid values
 */    
class IsRefCommand : public Condition {


public:
    IsRefCommand(std::string name, bt::Blackboard::Ptr blackboard = nullptr);  
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
