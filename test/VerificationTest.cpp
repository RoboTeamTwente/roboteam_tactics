#include <gtest/gtest.h>
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/bt.hpp"

namespace rtt {

class HasVerification : public Condition {
    public: 
    static VerificationMap required_params() {
            VerificationMap params;
            params["intparam"] = BBArgumentType::Int;
            return params;
    }
};

class MultiVerification : public Condition {
    public: 
    static VerificationMap required_params() {
            VerificationMap params;
            params["intparam"] = BBArgumentType::Int;
            params["doubleparam"] = BBArgumentType::Double;
            params["strparam"] = BBArgumentType::String;
            return params;
    }
};


class NoVerification : public Condition {};

class NamedVerification : public Condition {
    public:
    NamedVerification(std::string name) : Condition(name) {}
    static VerificationMap required_params() {
            VerificationMap params;
            params["floatparam"] = BBArgumentType::Float;
            return params;
    }
};

TEST(VerificationTest, do_test) {
    ASSERT_TRUE(CanVerify<HasVerification>::can_it);
    ASSERT_FALSE(CanVerify<NoVerification>::can_it);
    ASSERT_TRUE(CanVerify<MultiVerification>::can_it);
    ASSERT_TRUE(CanVerify<NamedVerification>::can_it);
    
    bt::Blackboard bb;
    bb.SetInt("intparam", 5);
    bb.SetString("strparam", "val");
    bb.SetFloat("test_cond_floatparam", .42f);
    
    auto ptr = std::make_shared<bt::Blackboard>(bb);
    
    ASSERT_TRUE(Leaf::validate_blackboard<HasVerification>(ptr));
    ASSERT_FALSE(Leaf::validate_blackboard<MultiVerification>(ptr));
    ASSERT_TRUE(Leaf::validate_blackboard<NoVerification>(ptr)); // no verification = no error
    ASSERT_FALSE(Leaf::validate_blackboard<NoVerification>(nullptr)); //... except for nullptr
    
    ASSERT_TRUE(Leaf::validate_blackboard<NamedVerification>(ptr, "test_cond"));
    ASSERT_FALSE(Leaf::validate_blackboard<NamedVerification>(ptr, "wrong_name"));
}

}