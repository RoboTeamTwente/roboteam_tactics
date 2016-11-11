#pragma once

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/verifier.h"
#include "ros/ros.h"

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
                case BBArgumentType::String:
                    this_valid = blackboard->HasString(key);
                    break;
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

    // Proxies that prefix an id for lookups in the global table.
    inline void SetBool(std::string key, bool value) { blackboard->SetBool(getPrefixedId(key), value); }
    inline bool GetBool(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) { 
        std::string real_key = getPrefixedId(key);
        switch (policy) {
            case BlackboardPolicy::GLOBAL_FIRST:
            if (blackboard->HasBool(real_key)) {
                return blackboard->GetBool(real_key);
            }
            case BlackboardPolicy::PRIVATE_ONLY: //fallthrough
            return private_bb->GetBool(real_key);
 
            case BlackboardPolicy::PRIVATE_FIRST:
            if (private_bb->HasBool(real_key)) {
                return private_bb->GetBool(real_key);
            }
            case BlackboardPolicy::GLOBAL_ONLY:
            return blackboard->GetBool(real_key);
            default:
            throw std::logic_error("Something really bad happened in GetBool...");
        }
    }
    inline bool HasBool(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const { 
        std::string real_key = getPrefixedId(key);
        if (policy == BlackboardPolicy::GLOBAL_FIRST || policy == BlackboardPolicy::PRIVATE_FIRST) {
            return blackboard->HasBool(real_key) || private_bb->HasBool(real_key);
        }
        if (policy == BlackboardPolicy::GLOBAL_ONLY) {
            return blackboard->HasBool(real_key);
        }
        return private_bb->HasBool(real_key);
    }

    inline void SetInt(std::string key, int value)  { blackboard->SetInt(getPrefixedId(key), value); }
    inline int GetInt(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) { 
        std::string real_key = getPrefixedId(key);
        switch (policy) {
            case BlackboardPolicy::GLOBAL_FIRST:
            if (blackboard->HasInt(real_key)) {
                return blackboard->GetInt(real_key);
            }
            case BlackboardPolicy::PRIVATE_ONLY: //fallthrough
            return private_bb->GetInt(real_key);
 
            case BlackboardPolicy::PRIVATE_FIRST:
            if (private_bb->HasInt(real_key)) {
                return private_bb->GetInt(real_key);
            }
            case BlackboardPolicy::GLOBAL_ONLY:
            return blackboard->GetInt(real_key);
            default:
            throw std::logic_error("Something really bad happened in GetInt...");
        }
    }
    inline int HasInt(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const { 
        std::string real_key = getPrefixedId(key);
        if (policy == BlackboardPolicy::GLOBAL_FIRST || policy == BlackboardPolicy::PRIVATE_FIRST) {
            return blackboard->HasInt(real_key) || private_bb->HasInt(real_key);
        }
        if (policy == BlackboardPolicy::GLOBAL_ONLY) {
            return blackboard->HasInt(real_key);
        }
        return private_bb->HasInt(real_key);
    }

    inline void SetFloat(std::string key, float value)  { blackboard->SetFloat(getPrefixedId(key), value); }
    inline float GetFloat(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) { 
        std::string real_key = getPrefixedId(key);
        switch (policy) {
            case BlackboardPolicy::GLOBAL_FIRST:
            if (blackboard->HasFloat(real_key)) {
                return blackboard->GetFloat(real_key);
            }
            case BlackboardPolicy::PRIVATE_ONLY: //fallthrough
            return private_bb->GetFloat(real_key);
 
            case BlackboardPolicy::PRIVATE_FIRST:
            if (private_bb->HasFloat(real_key)) {
                return private_bb->GetFloat(real_key);
            }
            case BlackboardPolicy::GLOBAL_ONLY:
            return blackboard->GetFloat(real_key);
            default:
            throw std::logic_error("Something really bad happened in GetFloat...");
        }
    }   
    inline float HasFloat(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const { 
        std::string real_key = getPrefixedId(key);
        if (policy == BlackboardPolicy::GLOBAL_FIRST || policy == BlackboardPolicy::PRIVATE_FIRST) {
            return blackboard->HasFloat(real_key) || private_bb->HasFloat(real_key);
        }
        if (policy == BlackboardPolicy::GLOBAL_ONLY) {
            return blackboard->HasFloat(real_key);
        }
        return private_bb->HasFloat(real_key);
    }
    
    inline void SetDouble(std::string key, double value)  { blackboard->SetDouble(getPrefixedId(key), value); }
    inline double GetDouble(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) { 
        std::string real_key = getPrefixedId(key);
        switch (policy) {
            case BlackboardPolicy::GLOBAL_FIRST:
            if (blackboard->HasDouble(real_key)) {
                return blackboard->GetDouble(real_key);
            }
            case BlackboardPolicy::PRIVATE_ONLY: //fallthrough
            return private_bb->GetDouble(real_key);
 
            case BlackboardPolicy::PRIVATE_FIRST:
            if (private_bb->HasDouble(real_key)) {
                return private_bb->GetDouble(real_key);
            }
            case BlackboardPolicy::GLOBAL_ONLY:
            return blackboard->GetDouble(real_key);
            default:
            throw std::logic_error("Something really bad happened in GetDouble...");
        }
    }
    inline bool HasDouble(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const { 
        std::string real_key = getPrefixedId(key);
        if (policy == BlackboardPolicy::GLOBAL_FIRST || policy == BlackboardPolicy::PRIVATE_FIRST) {
            return blackboard->HasDouble(real_key) || private_bb->HasDouble(real_key);
        }
        if (policy == BlackboardPolicy::GLOBAL_ONLY) {
            return blackboard->HasDouble(real_key);
        }
        return private_bb->HasDouble(real_key);
    }

    inline void SetString(std::string key, std::string value)  { blackboard->SetString(getPrefixedId(key), value); }
    inline std::string GetString(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) { 
        std::string real_key = getPrefixedId(key);
        switch (policy) {
            case BlackboardPolicy::GLOBAL_FIRST:
            if (blackboard->HasString(real_key)) {
                return blackboard->GetString(real_key);
            }
            case BlackboardPolicy::PRIVATE_ONLY: //fallthrough
            return private_bb->GetString(real_key);
 
            case BlackboardPolicy::PRIVATE_FIRST:
            if (private_bb->HasString(real_key)) {
                return private_bb->GetString(real_key);
            }
            case BlackboardPolicy::GLOBAL_ONLY:
            return blackboard->GetString(real_key);
            default:
            throw std::logic_error("Something really bad happened in GetString...");
        }
    }
    inline bool HasString(std::string key, BlackboardPolicy policy = DEFAULT_BB_POLICY) const { 
        std::string real_key = getPrefixedId(key);
        if (policy == BlackboardPolicy::GLOBAL_FIRST || policy == BlackboardPolicy::PRIVATE_FIRST) {
            return blackboard->HasString(real_key) || private_bb->HasString(real_key);
        }
        if (policy == BlackboardPolicy::GLOBAL_ONLY) {
            return blackboard->HasString(real_key);
        }
        return private_bb->HasString(real_key);
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
