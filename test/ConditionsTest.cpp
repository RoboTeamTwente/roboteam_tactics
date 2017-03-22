#include <gtest/gtest.h>
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/conditions/IHaveBall.h"

namespace rtt {

template<typename C>
inline void test(C condition) {
    for (auto world : condition.success_states()) {
        LastWorld::set(world);
        ASSERT_EQ(bt::Node::Status::Success, condition.Update());
    }
    for (auto world : condition.fail_states()) {
        LastWorld::set(world);
        ASSERT_EQ(bt::Node::Status::Failure, condition.Update());
    }
}

TEST(ConditionTests, IHaveBallTest) {
    bt::Blackboard bb;
    bb.SetInt("ihb_test_me", 0);
    bb.SetBool("ihb_test_our_team", true);
    test<IHaveBall>(IHaveBall("ihb_test", std::make_shared<bt::Blackboard>(bb)));
}

}
