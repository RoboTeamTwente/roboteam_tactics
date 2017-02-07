#pragma once

#include <cstdio>
#include <boost/optional.hpp>

#include <ros/message_forward.h>

namespace roboteam_msgs {

ROS_DECLARE_MESSAGE(BtDebugInfo);
ROS_DECLARE_MESSAGE(BtStatus);

}

#include "roboteam_tactics/utils/BtDebug.h"
#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/verifier.h"

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

    virtual Status Tick();

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
                    this_valid = blackboard->HasInt(key) || blackboard->HasInt(pair.first);
                    break;
                case BBArgumentType::Float:
                    this_valid = blackboard->HasFloat(key) || blackboard->HasFloat(pair.first);
                    break;
                case BBArgumentType::Double:
                    this_valid = blackboard->HasDouble(key) || blackboard->HasDouble(pair.first);
                    break;
                case BBArgumentType::Bool:
                    this_valid = blackboard->HasBool(key) || blackboard->HasBool(pair.first);
                    break;
                case BBArgumentType::String: {
                    this_valid = blackboard->HasString(key) || blackboard->HasString(pair.first);
                    std::string val = blackboard->HasString(key) ? blackboard->GetString(key) : blackboard->GetString(pair.first);
                    boost::optional<std::string> error = valid_string_opt<Impl>({key, val});
                    if ((bool) error) {
                        this_valid = false;
                        fprintf(stderr, "%s", error->c_str());
                    }
                    break;
                }
                default:
                throw std::logic_error("Incomplete switch statement in rtt::Leaf::validate_blackboard.");
            }
            if (!this_valid) {
                fprintf(stderr, "Blackboard verification error: no %s of type %s in blackboard \"%s\"", key.c_str(),
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
            fprintf(stderr, "Blackboard verification error: blackboard is null");
        }
        return blackboard != nullptr;
    }


    static const char* bbArgTypeName(BBArgumentType arg);

    Leaf(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    virtual ~Leaf();

    virtual Status Update();

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
    void SetBool(std::string key, bool value);
    bool GetBool(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY);

    bool HasBool(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const;

    void SetInt(std::string key, int value) ;
    int GetInt(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY);
    int HasInt(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const;

    void SetFloat(std::string key, float value) ;
    float GetFloat(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY);
    float HasFloat(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const;

    void SetDouble(std::string key, double value) ;
    double GetDouble(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY);
    bool HasDouble(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const;

    void SetString(std::string key, std::string value) ;
    std::string GetString(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY);
    bool HasString(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const;

    std::string getPrefixedId(std::string id) const;

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
