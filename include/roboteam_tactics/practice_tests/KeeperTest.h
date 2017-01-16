#pragma once

#include "roboteam_tactics/practice_tests/PracticeTest.h"

namespace rtt {

namespace practice {

class KeeperTest : public PracticeTest {
    virtual Config getConfig() override;

    virtual Result check(roboteam_msgs::World const & world) override;
} ;

} // namespace practice

} // namespace rtt
