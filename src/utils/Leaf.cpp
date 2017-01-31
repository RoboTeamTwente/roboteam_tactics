#include <string>

#include "roboteam_tactics/Leaf.h"

namespace rtt {

bt::Node::Status Leaf::Tick() {
    // Logic to, if enabled, send tree traces to the debug gui
    #ifdef RTT_ENABLE_BT_RQT_TRACE

    Status previousStatus = status;

    Status newStatus = bt::Leaf::Tick();

    roboteam_msgs::Blackboard bb;

    if (newStatus != bt::Node::Status::Running) {
        roboteam_msgs::BtDebugInfo::_type_type msgStatus;

        if (newStatus == bt::Node::Status::Success) {
            msgStatus = roboteam_msgs::BtStatus::SUCCESS;
        } else if (newStatus == bt::Node::Status::Failure) {
            msgStatus = roboteam_msgs::BtStatus::FAILURE;
        } else if (newStatus == bt::Node::Status::Invalid) {
            msgStatus = roboteam_msgs::BtStatus::INVALID;
        }

        RTT_SEND_RQT_BT_TRACE(name, roboteam_msgs::BtDebugInfo::TYPE_LEAF, msgStatus, bb);
    } else if (newStatus != previousStatus) {
        roboteam_msgs::BtDebugInfo::_type_type msgStatus = roboteam_msgs::BtStatus::STARTUP;
        RTT_SEND_RQT_BT_TRACE(name, roboteam_msgs::BtDebugInfo::TYPE_LEAF, msgStatus, bb);
    }

    return newStatus;

    #else

    return bt::Leaf::Tick();

    #endif
}

const char* Leaf::bbArgTypeName(BBArgumentType arg) {
    switch (arg) {
    case BBArgumentType::Int:
        return "int";
    case BBArgumentType::Float:
        return "float";
    case BBArgumentType::Double:
        return "double";
    case BBArgumentType::Bool:
        return "bool";
    case BBArgumentType::String:
        return "string";
    default:
        return "<<missing>>";
    }
}

Leaf::Leaf(std::string name, bt::Blackboard::Ptr blackboard)
            : bt::Leaf(blackboard)
            , name{name}
            {}

Leaf::~Leaf() {}

bt::Node::Status Leaf::Update() {
    return Status::Invalid;
}

// Proxies that prefix an id for lookups in the global table.
void Leaf::SetBool(std::string key, bool value) { blackboard->SetBool(getPrefixedId(key), value); }
bool Leaf::GetBool(std::string key, BlackboardPolicy policy) {
    return GetVar<bool, &bt::Blackboard::HasBool, &bt::Blackboard::GetBool>(key, policy);
}

bool Leaf::HasBool(std::string key, BlackboardPolicy policy) const {
    return HasVar<bool, &bt::Blackboard::HasBool>(key, policy);
}

void Leaf::SetInt(std::string key, int value)  { blackboard->SetInt(getPrefixedId(key), value); }
int Leaf::GetInt(std::string key, BlackboardPolicy policy) {
    return GetVar<int, &bt::Blackboard::HasInt, &bt::Blackboard::GetInt>(key, policy);
}
int Leaf::HasInt(std::string key, BlackboardPolicy policy) const {
    return HasVar<int, &bt::Blackboard::HasInt>(key, policy);
}

void Leaf::SetFloat(std::string key, float value)  { blackboard->SetFloat(getPrefixedId(key), value); }
float Leaf::GetFloat(std::string key, BlackboardPolicy policy) {
    return GetVar<float, &bt::Blackboard::HasFloat, &bt::Blackboard::GetFloat>(key, policy);
}
float Leaf::HasFloat(std::string key, BlackboardPolicy policy) const {
    return HasVar<float, &bt::Blackboard::HasFloat>(key, policy);
}

void Leaf::SetDouble(std::string key, double value)  { blackboard->SetDouble(getPrefixedId(key), value); }
double Leaf::GetDouble(std::string key, BlackboardPolicy policy) {
    return GetVar<double, &bt::Blackboard::HasDouble, &bt::Blackboard::GetDouble>(key, policy);
}
bool Leaf::HasDouble(std::string key, BlackboardPolicy policy) const {
    return HasVar<double, &bt::Blackboard::HasDouble>(key, policy);
}

void Leaf::SetString(std::string key, std::string value)  { blackboard->SetString(getPrefixedId(key), value); }
std::string Leaf::GetString(std::string key, BlackboardPolicy policy) {
    return GetVar<std::string, &bt::Blackboard::HasString, &bt::Blackboard::GetString>(key, policy);
}
bool Leaf::HasString(std::string key, BlackboardPolicy policy) const {
    return HasVar<std::string, &bt::Blackboard::HasString>(key, policy);
}

std::string Leaf::getPrefixedId(std::string id) const {
    if (name.empty()) {
        return id;
    }

    return name + "_" + id;
}

} // rtt
