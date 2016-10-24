#pragma once

#include <map>
#include <type_traits>

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
class CanVerify {
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
     * called, and no instance of struct Check is created. Both of those things are
     * impossible.
     */
    static constexpr bool can_it = decltype(test<L>(nullptr))::value;
};    

}
