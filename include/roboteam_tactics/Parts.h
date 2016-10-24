#pragma once

#include "ros/ros.h"

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/Aggregator.h"
#include "roboteam_msgs/World.h"
#include <map>
#include <type_traits>
#include <vector>

namespace rtt {
    
/**
* Possible bt::Blackboard value types. For use in Blackboard verification.
*/
enum class BBArgumentType { Int, Float, Double, Bool, String };

/**
* Type used in verification of Blackboards.
*/
typedef std::map<std::string, BBArgumentType> VerificationMap;
    
/**
 * @brief Checks whether a given {L}eaf is able to verify a Blackboard argument.
 * See the documentation of rtt::Leaf::validate_blackboard for the requirements.
 */
template<typename L>
class CanVerify final {
    private:
    /** 
     * Helper to check whether a type and value match. 
     * @deprecated This struct only has a forward declaration, and so cannot be used.
     */
    template<typename T, T> struct ValueHasType;
    
    /** 
     * @brief 'True' overload. This 'function' can only be selected if 
     *  T has a static function with the required signature.
     * @deprecated This function has no implementation, and so must not be called.
     */
    template<typename T>
    static std::true_type test(ValueHasType<VerificationMap(*)(), &T::required_params>*);
    
    /** 
     * @brief 'False' overload. This alternative is selected only if the above failed.
     * That is, if no function with the required signature exists in T.
     * @deprecated This function has no implementation, and so must not be called.
     */
    template<typename T>
    static std::false_type test(...);

    /** @deprecated CanVerify cannot be initialized */
    CanVerify() { throw new std::logic_error("CanVerify cannot be initialized."); } 
    public:
    
    /**
     * This selects one of the overloaded test() functions and checks the
     * return type. It is important to notice that the test function is not actually
     * called, and no instance of struct ValueHasType is created. Both of those things are
     * impossible.
     */
    static constexpr bool can_it = decltype(test<L>(nullptr))::value;
};    
    
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
    static constexpr bool validate_blackboard(typename std::enable_if<CanVerify<Impl>::can_it, bt::Blackboard::Ptr>::type blackboard, 
                                              std::string name = "") {
        if (blackboard == nullptr) return false;
        VerificationMap required = Impl::required_params();
        for (const auto& pair : required) {
            bool valid = true; // need initialized value due to constexpr
            std::string key = name.empty() ? pair.first : name + "_" + pair.first;
            switch (pair.second) {
                case BBArgumentType::Int:
                    valid = blackboard->HasInt(key);
                    break;
                case BBArgumentType::Float:
                    valid = blackboard->HasFloat(key);
                    break;
                case BBArgumentType::Double:
                    valid = blackboard->HasDouble(key);
                    break;
                case BBArgumentType::Bool:
                    valid = blackboard->HasBool(key);
                    break;
                case BBArgumentType::String:
                    valid = blackboard->HasString(key);
                    break;
                default:
                throw std::logic_error("Incomplete switch statement in rtt::Leaf::validate_blackboard.");
            }
            if (!valid) return false;
        }
        return true;
    }
    
    /**
     * @brief Overload for Leafs which do not provide a required_params() function. This only performs a null check on the Blackboard.
     */
    template<typename Impl>
    static constexpr bool validate_blackboard(typename std::enable_if<!CanVerify<Impl>::can_it, bt::Blackboard::Ptr>::type blackboard,
                                              std::string name = "") {
        return blackboard != nullptr;
    }
    
    
    Leaf(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr)
            : bt::Leaf(blackboard)
            , name{name}
            {}
    virtual ~Leaf() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    bt::Blackboard::Ptr private_bb = std::make_shared<bt::Blackboard>();
    const std::string name;

    // Proxies that prefix an id for lookups in the global table.
    inline void SetBool(std::string key, bool value) { blackboard->SetBool(getPrefixedId(key), value); }
    inline bool GetBool(std::string key) { return blackboard->GetBool(getPrefixedId(key)); }
    inline bool HasBool(std::string key) const { return blackboard->HasBool(getPrefixedId(key)); }

    inline void SetInt(std::string key, int value)  { blackboard->SetInt(getPrefixedId(key), value); }
    inline int GetInt(std::string key) { return blackboard->GetInt(getPrefixedId(key)); }
    inline bool HasInt(std::string key) const  { return blackboard->HasInt(getPrefixedId(key)); }

    inline void SetFloat(std::string key, float value)  { blackboard->SetFloat(getPrefixedId(key), value); }
    inline float GetFloat(std::string key) { return blackboard->GetFloat(getPrefixedId(key)); }
    inline bool HasFloat(std::string key) const  { return blackboard->HasFloat(getPrefixedId(key)); }

    inline void SetDouble(std::string key, double value)  { blackboard->SetDouble(getPrefixedId(key), value); }
    inline double GetDouble(std::string key) { return blackboard->GetDouble(getPrefixedId(key)); }
    inline bool HasDouble(std::string key) const  { return blackboard->HasDouble(getPrefixedId(key)); }

    inline void SetString(std::string key, std::string value)  { blackboard->SetString(getPrefixedId(key), value); }
    inline std::string GetString(std::string key) { return blackboard->GetString(getPrefixedId(key)); }
    inline bool HasString(std::string key) const  { return blackboard->HasString(getPrefixedId(key)); }
    
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

class Skill : public Leaf {
public:

    Skill(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(name, blackboard)
            , n{n}
            {}
    virtual ~Skill() {}

    virtual Status Update() {
        return Status::Invalid;
    }

    ros::NodeHandle n;
} ;

class Condition : public Leaf {
	public:
	Condition(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(name, blackboard)
            {}
    virtual ~Condition() {}

    virtual Status Update() {
        return Status::Invalid;
    }
    
    /**
     * @brief Returns a set of world states in which this Condition should succeed.
     * For use in testing. See test/ConditionsTest.cpp
     */
    virtual std::vector<roboteam_msgs::World> success_states() const {
        return std::vector<roboteam_msgs::World>();
    }
    
    /**
     * @brief Returns a set of world states in which this Condition should fail.
     * For use in testing. See test/ConditionsTest.cpp
     */
    virtual std::vector<roboteam_msgs::World> fail_states() const {
        return std::vector<roboteam_msgs::World>();
    }    
} ;

class Role : public bt::BehaviorTree {
    public:

    Role (int robotId, bt::Blackboard::Ptr blackboard = nullptr) {
        SetSharedBlackboard(blackboard);
    }
    virtual ~Role() {}

    virtual Status Update() {
        return Status::Invalid;
    }
} ;

class Tactic : public bt::Leaf {
    public:
    Tactic(bt::Blackboard::Ptr blackboard = nullptr)
            : Leaf(blackboard)
            {}
    virtual ~Tactic() {}

    virtual void distribute_roles() {}
    virtual bt::Node::Status update_roles() {
        return bt::Node::Status::Invalid;
    }
} ;

class Strategy : public bt::BehaviorTree {
    public:
    Strategy() {}
    virtual ~Strategy() {}

    virtual Status Update() {
        return Status::Invalid;
    }
} ;

}
