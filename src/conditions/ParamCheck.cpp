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
    if (!ros::param::get(signal, signalValue)) {
    	return bt::Node::Status::Failure;
    }
    if (compare_values(mode, signalValue, bb_value)) {
        RTT_DEBUG("Success!\n");
        return bt::Node::Status::Success;
    } else {
        RTT_DEBUG("Failure!\n");
        return bt::Node::Status::Failure;
    }
}

BBArgumentType ParamCheck::deduceType() {
	if (HasString("typeOverride")) {
		std::string type = GetString("typeOverride");
		if (type == "string") {
			return BBArgumentType::String;
		} else if (type == "int") {
			return BBArgumentType::Int;
		} else if (type == "double") {
			return BBArgumentType::Double;
		} else if (type == "bool") {
			return BBArgumentType::Bool;
		}
		ROS_ERROR("ParamCheck::deduceType: Invalid typeOverride: %s", type.c_str());
		throw std::invalid_argument("ParamCheck::deduceType:: Invalid typeOverride");
	} else if (HasString("value")) {
		return BBArgumentType::String;
	} else if (HasInt("value")) {
		return BBArgumentType::Int;
	} else if (HasDouble("value")) {
		return BBArgumentType::Double;
	} else if (HasBool("value")) {
		return BBArgumentType::Bool;
	}
	ROS_ERROR("ParamCheck::deduceType: No 'value' parameter of any type");
	throw std::invalid_argument("ParamCheck::deduceType: No 'value' parameter of any type");
}

bt::Node::Status ParamCheck::Update() {
    std::string signal = "/signal_" + GetString("signal");
    std::string mode = GetString("mode");
    std::string argType;

    BBArgumentType type = deduceType();
    RTT_DEBUGLN("Type: %d", static_cast<int>(type));
    switch (type) {
    case BBArgumentType::String:
        return check_param(signal, mode, std::string(), GetString("value"));
    case BBArgumentType::Double:
        return check_param(signal, mode, 0.0, GetDouble("value"));
    case BBArgumentType::Int:
        return check_param(signal, mode, 0, GetInt("value"));
    case BBArgumentType::Bool:
        return check_param(signal, mode, false, GetBool("value"));
    default:
        // This is impossible. deduceType should have thrown an std::invalid_argument.
    	throw std::logic_error("ParamCheck::Update: deduceType did not fail when it should have.");
    }
}

} // rtt
