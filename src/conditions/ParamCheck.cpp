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
    if (compare_values(mode, signalValue, bb_value)) {
        RTT_DEBUG("Success!\n");
        return bt::Node::Status::Success;
    } else {
        RTT_DEBUG("Failure!\n");
        return bt::Node::Status::Failure;
    }
}

bt::Node::Status ParamCheck::Update() {
    std::string signal = "/signal_" + GetString("signal");
    std::string mode = GetString("mode");

    std::string argType;
    if (HasString("value")) {
        return check_param(signal, mode, std::string(""), GetString("value"));
    } else if (HasDouble("value")) {
        return check_param(signal, mode, 0.0, GetDouble("value"));
    } else if (HasInt("value")) {
        return check_param(signal, mode, 0, GetInt("value"));
    } else if (HasBool("value")) {
        return check_param(signal, mode, false, GetBool("value"));
    } else {
        // If no param is set we cannot properly check the ros param. Hence, we fail.
        return bt::Node::Status::Failure;
    }
}

} // rtt
