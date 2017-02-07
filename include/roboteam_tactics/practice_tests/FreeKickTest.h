#pragma once

#include "roboteam_tactics/practice_tests/PracticeTest.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/tactics/FreeKickDefenceTactic.h"

namespace rtt {
namespace practice {

class FreeKickTest : public PracticeTest {
 
public:

    FreeKickTest();

    time_point testStart;

    boost::optional<Config> getConfig(Side side, std::vector<RobotID> const & ourRobots, 
            roboteam_msgs::GeometryFieldSize const & fieldGeom) override;

    void beforeTest(roboteam_msgs::World const & world) override;

    Result check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom) override;

    std::string testName() override;   
    
private:
    FreeKickDefenceTactic tactic;
    
};

}
}