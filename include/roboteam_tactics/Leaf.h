#pragma once

#include <boost/optional.hpp>
#include <ros/ros.h>

#include "roboteam_tactics/utils/BtDebug.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/verifier.h"
#include "roboteam_msgs/BtDebugInfo.h"
#include "roboteam_msgs/BtStatus.h"

namespace rtt {

enum class BlackboardPolicy {
    GLOBAL_FIRST,
    PRIVATE_FIRST,
    GLOBAL_ONLY,
    PRIVATE_ONLY
};

constexpr BlackboardPolicy DEFAULT_BB_POLICY = BlackboardPolicy::GLOBAL_FIRST;

class Leaf : public bt::Leaf {
public:

    Status Tick() {
        #ifdef RTT_ENABLE_BT_RQT_TRACE

        Status previousStatus = status;

        Status newStatus = bt::Leaf::Tick();

        roboteam_msgs::Blackboard bb;

        if (newStatus == bt::Node::Status::Success) {
            RTT_SEND_RQT_BT_TRACE(name, roboteam_msgs::BtStatus::SUCCESS, bb);
        } else if (newStatus == bt::Node::Status::Failure) {
            RTT_SEND_RQT_BT_TRACE(name, roboteam_msgs::BtStatus::FAILURE, bb);
        } else if (newStatus == bt::Node::Status::Invalid) {
            RTT_SEND_RQT_BT_TRACE(name, roboteam_msgs::BtStatus::INVALID, bb);
        } else if (previousStatus != bt::Node::Status::Running) {
            RTT_SEND_RQT_BT_TRACE(name, roboteam_msgs::BtStatus::STARTUP, bb);
        } else {
            // Don't send anything
        }
            
        return newStatus;

        #else   

        return bt::Leaf::Tick();

        #endif
    }

    template<typename Impl>
    static boost::optional<std::string> valid_string_opt(typename std::enable_if<HasStringOptions<Impl>::value,
                                                                  std::pair<std::string, std::string>>::type key_value) {
        std::vector<std::string> possible = Impl::valid_options(key_value.first);
        if (!possible.empty() && std::find(possible.begin(), possible.end(), key_value.second()) == possible.end()) {
            std::string msg = "Blackboard verification error: Option '%s' is invalid for key '%s'. Possible options are: [";
            bool first = true;
            for (const std::string& opt : possible) {
                if (!first) {
                    msg += ", ";
                } else {
                    first = false;
                }
                msg += opt;
            }
            return boost::optional<std::string>(msg + "].");
        }
        return boost::optional<std::string>();
    }

    template<typename Impl>
    static boost::optional<std::string> valid_string_opt(typename std::enable_if<!HasStringOptions<Impl>::value,
                                                                  std::pair<std::string, std::string>>::type key_value) {
        return boost::optional<std::string>();
    }

    /**
     * @brief Tests whether a certain Blackboard instance contains the required variables for a certain type of Leaf (Impl).
     * Not all Leafs neccessarily have the capability to validate Blackboards, in which case only a null check will be performed.
     * To enable validation, create a function in your Leaf with this (public) signature:
     *
     *          static VerificationMap required_params(); // VerificationMap = std::map<std::string, BBArgumentType>
     *
     * This function should return name-type pairs for the variables the Leaf's constructor requires.
     * If no name parameter is passed, this function will check for variables of the names exactly as specified
     * in the VerificationMap. Otherwise, it will augment the names in the same way rtt::Leaf's GetX, HasX and SetX methods do.
     * @return Whether or not the given Blackboard satisfies the requirements of the Leaf specified as the template parameter.
     */
    template<typename Impl>
    static bool validate_blackboard(typename std::enable_if<CanVerify<Impl>::can_it, bt::Blackboard::Ptr>::type blackboard,
                                              std::string name = "") {
        if (blackboard == nullptr) return false;
        VerificationMap required = Impl::required_params();
        bool valid = true;

        for (const auto& pair : required) {
            std::string key = name.empty() ? pair.first : name + "_" + pair.first;
            bool this_valid = true;
            switch (pair.second) {
                case BBArgumentType::Int:
                    this_valid = blackboard->HasInt(key);
                    break;
                case BBArgumentType::Float:
                    this_valid = blackboard->HasFloat(key);
                    break;
                case BBArgumentType::Double:
                    this_valid = blackboard->HasDouble(key);
                    break;
                case BBArgumentType::Bool:
                    this_valid = blackboard->HasBool(key);
                    break;
                case BBArgumentType::String: {
                    this_valid = blackboard->HasString(key);
                    boost::optional<std::string> error = valid_string_opt<Impl>({key, blackboard->GetString(key)});
                    if ((bool) error) {
                        this_valid = false;
                        ROS_ERROR("%s", error->c_str());
                    }
                    break;
                }
                default:
                throw std::logic_error("Incomplete switch statement in rtt::Leaf::validate_blackboard.");
            }
            if (!this_valid) {
                ROS_ERROR("Blackboard verification error: no %s of type %s in blackboard \"%s\"", key.c_str(),
                    bbArgTypeName(pair.second), name.c_str());
                valid = false;
            }
        }
        return valid;
    }

    /**
     * @brief Overload for Leafs which do not provide a required_params() function. This only performs a null check on the Blackboard.
     */
    template<typename Impl>
    static bool validate_blackboard(typename std::enable_if<!CanVerify<Impl>::can_it, bt::Blackboard::Ptr>::type blackboard,
                                              std::string name = "") {

        if (blackboard == nullptr) {
            ROS_ERROR("Blackboard verification error: blackboard is null");
        }
        return blackboard != nullptr;
    }


    static const char* bbArgTypeName(BBArgumentType arg) {
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

    Leaf(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr)
            : bt::Leaf(blackboard)
            , name{name}
            {}
    virtual ~Leaf() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    const std::string name;

    private:
    template<typename T, bool (bt::Blackboard::*Checker)(std::string) const, T (bt::Blackboard::*Getter)(std::string)>
    T GetVar(const std::string& key, BlackboardPolicy policy) const {
        std::string global_key = getPrefixedId(key);
        switch (policy) {
            case BlackboardPolicy::GLOBAL_FIRST:
            if ((*blackboard.*Checker)(global_key)) {
                return (*blackboard.*Getter)(global_key);
            }
            case BlackboardPolicy::PRIVATE_ONLY: //fallthrough
            return (*private_bb.*Getter)(key);

            case BlackboardPolicy::PRIVATE_FIRST:
            if ((*private_bb.*Checker)(key)) {
                return (*private_bb.*Getter)(key);
            }
            case BlackboardPolicy::GLOBAL_ONLY:
            return (*blackboard.*Getter)(global_key);
            default:
            throw std::logic_error("Something really bad happened in GetVar...");
        }
    }

    template<typename T, bool (bt::Blackboard::*Checker)(std::string) const>
    bool HasVar(const std::string& key, BlackboardPolicy policy) const {
        std::string real_key = getPrefixedId(key);
        if (policy == BlackboardPolicy::GLOBAL_FIRST || policy == BlackboardPolicy::PRIVATE_FIRST) {
            return (*blackboard.*Checker)(real_key) || (*private_bb.*Checker)(key);
        }
        if (policy == BlackboardPolicy::GLOBAL_ONLY) {
            return (*blackboard.*Checker)(real_key);
        }
        return (*private_bb.*Checker)(key);
    }

    public:
    // Proxies that prefix an id for lookups in the global table.
    inline void SetBool(std::string key, bool value) { blackboard->SetBool(getPrefixedId(key), value); }
    inline bool GetBool(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) {
        return GetVar<bool, &bt::Blackboard::HasBool, &bt::Blackboard::GetBool>(key, policy);
    }

    inline bool HasBool(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const {
        return HasVar<bool, &bt::Blackboard::HasBool>(key, policy);
    }

    inline void SetInt(std::string key, int value)  { blackboard->SetInt(getPrefixedId(key), value); }
    inline int GetInt(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) {
        return GetVar<int, &bt::Blackboard::HasInt, &bt::Blackboard::GetInt>(key, policy);
    }
    inline int HasInt(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const {
        return HasVar<int, &bt::Blackboard::HasInt>(key, policy);
    }

    inline void SetFloat(std::string key, float value)  { blackboard->SetFloat(getPrefixedId(key), value); }
    inline float GetFloat(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) {
        return GetVar<float, &bt::Blackboard::HasFloat, &bt::Blackboard::GetFloat>(key, policy);
    }
    inline float HasFloat(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const {
        return HasVar<float, &bt::Blackboard::HasFloat>(key, policy);
    }

    inline void SetDouble(std::string key, double value)  { blackboard->SetDouble(getPrefixedId(key), value); }
    inline double GetDouble(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) {
        return GetVar<double, &bt::Blackboard::HasDouble, &bt::Blackboard::GetDouble>(key, policy);
    }
    inline bool HasDouble(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const {
        return HasVar<double, &bt::Blackboard::HasDouble>(key, policy);
    }

    inline void SetString(std::string key, std::string value)  { blackboard->SetString(getPrefixedId(key), value); }
    inline std::string GetString(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) {
        return GetVar<std::string, &bt::Blackboard::HasString, &bt::Blackboard::GetString>(key, policy);
    }
    inline bool HasString(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const {
        return HasVar<std::string, &bt::Blackboard::HasString>(key, policy);
    }

    std::string getPrefixedId(std::string id) const {
        if (name.empty()) {
            return id;
        }

        return name + "_" + id;
    }

    protected:

    /**
     * @brief Asserts that a Blackboard is valid for this Leaf. Leaf implementors are
     * encouraged to call this method from the constructor before using the Blackboard.
     * @param name The (optional) name of the Leaf. See rtt::Leaf::validate_blackboard
     * for details.
     */
    template<typename Impl>
    void assert_valid(std::string name = "") const {
        if (!validate_blackboard<Impl>(blackboard, name)) {
            throw std::invalid_argument("Blackboard verification failed.");
        }
    }
} ;

}
