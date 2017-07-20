#pragma once

#include "roboteam_tactics/Leaf.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

/**
 * \class AimingAtOpponent
 * \brief See YAML
 */
 /*
  * Descr: Checks whether a robot can see a certain point
  * Params:
  *   - ROBOT_ID:
  *       Type: Int
  *       Descr: The robot to check for
  */
class AimingAtOpponent : public Condition {
  
    public:
    static VerificationMap required_params() {
        VerificationMap params;
        params["me"] = BBArgumentType::Int;
        params["x_coor"] = BBArgumentType::Double;
        params["y_coor"] = BBArgumentType::Double;
        params["check_move"] = BBArgumentType::Bool;
        return params;
    }
    
    AimingAtOpponent(std::string name, bt::Blackboard::Ptr blackboard);
    Status Update() override;
    std::string node_name() override { return "AimingAtOpponent"; }
    
    protected:
    double threshold_dist;
    
};

}
