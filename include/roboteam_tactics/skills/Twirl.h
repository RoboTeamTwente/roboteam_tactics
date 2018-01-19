#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class Twirl
 * \brief See YAML
 */
/*
 * Descr: Twirls the robot at eye blazing speed.
 * Params:
 *   - ROBOT_ID:
 *         Type: Int
 *         Descr: The id of the robot to stop
 *   - speed:
 *         Type: Double
 *         Descr: Number if rotations per second
 *         Default: 0.5
 */
class Twirl final : public Skill {

public:
    Twirl(std::string name = "", bt::Blackboard::Ptr = nullptr);
    Status Update() override;
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["repeat"] = BBArgumentType::Int;
        return params;
    }
    
    std::string node_name() override { return "Twirl"; }
private:
    int count;
};
    
}
