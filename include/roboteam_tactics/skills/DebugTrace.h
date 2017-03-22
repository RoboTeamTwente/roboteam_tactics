#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

/**
 * \class DebugTrace
 * \brief See YAML
 */
/*
 * Descr: Does nothing for a set time
 * Params:
 *   - ms:
 *       Type: Int
 *       Descr: The amount of milliseconds to wait
 */
class DebugTrace : public Skill {
public:
	DebugTrace(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();
	Status Update();
    
    static VerificationMap required_params() {
        VerificationMap params;
        params["ms"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() { return "DebugTrace"; }
private:

    time_point start;
};


} // rtt
