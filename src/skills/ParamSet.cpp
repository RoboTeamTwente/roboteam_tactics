#include "ros/ros.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/ParamSet.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG ParamSet

namespace rtt {

ParamSet::ParamSet(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard) { }

bt::Node::Status ParamSet::Update() {
    std::string signal = "signal_" + GetString("signal");
    std::string value = GetString("value");

    std::string argType;
    if (value == "true") {
        argType = "bool";
    } else if (value == "false") {
        argType = "bool";
    } else if (value.find(".") != std::string::npos) {
        argType = "double";
    } else if (rtt::is_digits(value)) {
        argType = "int";
    } else {
        argType = "string";
    }

    try {
        if (argType == "string") {
            ros::param::set(signal, value);
        } else if (argType == "int") {
            ros::param::set(signal, std::stoi(value));
        } else if (argType == "double") {
            ros::param::set(signal, std::stod(value));
        } else if (argType == "bool") {
            ros::param::set(signal, value == "true");
        } else {
            std::cout << "Unknown arg type: " << argType << "\n";
        }
    } catch (...) {
        std::string signalName = GetString("signal");
        ROS_ERROR("Could not set signal param to value! Signal: %s, value: %s", signalName.c_str(), value.c_str());
        return Status::Failure;
    }

    RTT_DEBUG("Param set! Signal: %s, value: %s\n", signal.c_str(), value.c_str());

    return Status::Success;
}

} // rtt
