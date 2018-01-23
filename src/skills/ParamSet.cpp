#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/ParamSet.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG ParamSet

namespace rtt {

RTT_REGISTER_SKILL(ParamSet);

ParamSet::ParamSet(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) { }

bt::Node::Status ParamSet::Update() {
    std::string signal = "/signal_" + GetString("signal");
    std::string value;
    std::string argType;
    if (HasString("stringValue")) {
        value = GetString("stringValue");
        argType = "string";
        ros::param::set(signal, value);
    } else if (HasDouble("doubleValue")) {
        double val = GetDouble("doubleValue");
        ros::param::set(signal, val);
        argType = "double";
        value = std::to_string(val);
    } else if (HasInt("intValue")) {
        int val = GetInt("intValue");
        ros::param::set(signal, val);
        argType = "value";
        value = std::to_string(val);
    } else if (HasBool("boolValue")) {
        if (GetBool("boolValue")) {
            value = "true";
            ros::param::set(signal, true);
        } else {
            ros::param::set(signal, false);
            value = "false";
        }

        argType = "bool";
    } else {
        // No value is set. Simply make the param empty and exit.
        ros::param::del(signal);
        RTT_DEBUGLN("No value set for param %s", signal.c_str());
        return Status::Success;
    }

    RTT_DEBUG("Param set! Signal: %s, value: %s, value type: %s\n", signal.c_str(), value.c_str(), argType.c_str());

    return Status::Success;
}

} // rtt
