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
 * Descr: Prints it's name to standard out everytime it updates.
 * Params:
 *   - doReturn:
 *      Type: String
 *      Descr: What it should do after an update.
 *      Can be:
 *         Success: Return success upon finishing
 *         Failure: Return failure upon finishing
 *         Running: Return running upon finishing
 *      Default: Running
 *   - msg:
 *      Type: String
 *      Descr: A message that will be appended to the print if it is present
 *      
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
