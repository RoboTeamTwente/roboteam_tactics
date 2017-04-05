#pragma once

#include "roboteam_tactics/Parts.h"

namespace rtt {

/**
 * \class Freeze
 * \brief See YAML
 */
/*
 * Descr: Stops a robot from moving, possibly repeatedly
 * Params:
 *   ROBOT_ID:
 *     Type: Int
 *     Descr: The id of the robot to stop
 *   repeat:
 *     Type: Int
 *     Descr: |
 *       The amount of times to repeat the stop command. Set to zero to send it only once.
 *       If this number is set to a negative value, this skill will always return Running.
 *       (Or at least, it will do so between 2^31 and 2^32 times)
 */
class Freeze final : public Skill {

public:
    Freeze(std::string name = "", bt::Blackboard::Ptr = nullptr);
    void Initialize() override;
    Status Update() override;
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["repeat"] = BBArgumentType::Int;
        return params;
    }
    
    std::string node_name() override { return "Freeze"; }
private:
    int count;
};
    
}
