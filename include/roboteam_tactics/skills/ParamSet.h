#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

/**
 *
 * Descr: >
 *     Sets a param. If the value param is not present in any type,
 *     the signal is deleted from the ros param server. Only one value will be red
 *
 *     @Idea Ofcourse maybe introduce the robot param here as well.
 *     Or use ROBOT_ID?
 * 
 * Params:
 *    - signal:
 *        Type: String
 *        Descr: The signal to set
 *
 *    - stringValue:
 *        Type: String
 *        Descr: >
 *            The string value to set.
 *
 *    - doubleValue:
 *        Type: Double
 *        Descr: >
 *            The double value to set.
 *
 *    - intValue:
 *        Type: Int
 *        Descr: >
 *            The int value to set.
 *
 *    - boolValue:
 *        Type: Bool
 *        Descr: >
 *            The boolean value to set.
 */
class ParamSet : public Skill {
public:
	ParamSet(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
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
