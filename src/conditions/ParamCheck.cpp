#include <iostream>
#include <ros/ros.h>

#include "roboteam_tactics/conditions/ParamCheck.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

// To make sure RTT_DEBUG logs everything on ParamCheck
#define RTT_CURRENT_DEBUG_TAG ParamCheck

namespace rtt {

RTT_REGISTER_CONDITION(ParamCheck);

ParamCheck::ParamCheck(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}

template<
    typename T
>
bool compare_values(std::string mode, const T &left, const T &right) {
    if (mode == "lt") {
        return left < right;
    } else if (mode == "gt") {
        return left > right;
    } else if (mode == "eq") {
        return left == right;
    } else if (mode == "neq") {
        return left != right;
    } else if (mode == "leq") {
        return left <= right;
    } else { // mode == "geq"
        return left >= right;
    }
}

template<
    typename T
>
bt::Node::Status check_param(std::string signal, std::string mode, const T &default_val, const T &bb_value) {
    T signalValue = default_val;
    ros::param::get(signal, signalValue);

    // std::cout << "Signal: " << signalValue << "\n";
    if (compare_values(mode, signalValue, bb_value)) {
        RTT_DEBUG("Success!\n");
        return bt::Node::Status::Success;
    } else {
        RTT_DEBUG("Failure!\n");
        return bt::Node::Status::Failure;
    }
}

bt::Node::Status ParamCheck::Update() {
    std::string signal = "signal_" + GetString("signal");
    std::string mode = GetString("mode");
    std::string value = GetString("value");

    // Determine type
    std::string valueType = "";
    if (value == "true") {
        valueType = "bool";
    } else if (value == "false") {
        valueType = "bool";
    } else if (value.find(".") != std::string::npos) {
        valueType = "double";
    } else if (is_digits(value)) {
        valueType = "int";
    } else {
        valueType = "string";
    }

    // Parse type and check it
    if (valueType == "bool") {
        ROS_INFO_STREAM("it's a bool!");
        if ((mode != "eq") && (mode != "neq")) {
            ROS_ERROR("Invalid mode supplied for type bool! Value: %s, mode: %s, signal: %s", value.c_str(), mode.c_str(), signal.c_str());
            return Status::Failure;
        }

        bool boolValue = value == "true" ? true : false;

        return check_param(signal, mode, false, boolValue);
    } else if (valueType == "double") {
        double doubleValue;
        try {
            doubleValue = std::stod(value);
        } catch (...) {
            ROS_ERROR("Could not parse signal value into double! Value: %s, mode: %s, signal: %s", value.c_str(), mode.c_str(), signal.c_str());
            return Status::Failure;
        }

        return check_param(signal, mode, 0.0, doubleValue);
    } else if (valueType == "int") {
        int intValue;
        try {
            intValue = std::stoi(value);
        } catch (...) {
            ROS_ERROR("Could not parse signal value into double! Value: %s, mode: %s, signal: %s", value.c_str(), mode.c_str(), signal.c_str());
            return Status::Failure;
        }

        return check_param(signal, mode, 0, intValue);
    } else { // valueType == "string"
        if ((mode != "eq") && (mode != "neq")) {
            ROS_ERROR("[ParamCheck] Invalid mode supplied for type string! Value: %s, mode: %s, signal: %s", value.c_str(), mode.c_str(), signal.c_str());
            return Status::Failure;
        }

        return check_param(signal, mode, std::string(""), value);
    }

    // In case of programmer error return failure
    ROS_ERROR("None of the types matched: error! Value: %s, mode: %s, signal: %s", value.c_str(), mode.c_str(), signal.c_str());

    return Status::Failure;
}

} // rtt
