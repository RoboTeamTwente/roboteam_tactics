#pragma once

#include <boost/optional.hpp>

#include "roboteam_tactics/practice_tests/PracticeTest.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

namespace practice {

class KeeperTest : public PracticeTest {
public:
    time_point testStart;

    boost::optional<Config> getConfig(
        Side side, 
        std::vector<RobotID> ourRobots, 
        roboteam_msgs::GeometryFieldSize fieldGeom
        ) override;

    void beforeTest(roboteam_msgs::World const & world) override;

    Result check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom) override;

    std::string testName() override;
} ;

} // namespace practice

} // namespace rtt
