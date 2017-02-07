#include "roboteam_tactics/practice_tests/FreeKickTest.h"
#include "roboteam_tactics/practice_tests/TestSetup.h"
#include "roboteam_tactics/utils/RobotDealer.h"

namespace rtt {
namespace practice {
    
FreeKickTest::FreeKickTest() : tactic("tactic", nullptr) {}
    
boost::optional<Config> FreeKickTest::getConfig(Side side, const std::vector<RobotID>& ourRobots,
 const roboteam_msgs::GeometryFieldSize& fieldGeom) {
    if (ourRobots.size() < 3) return boost::none;
    
    return boost::optional<Config>(SetupBuilder::builder()
        ->with_bot ({ourRobots.at(0), true}, {4.3, 0,   165}) // Keeper
        ->with_bot ({ourRobots.at(1), true}, {3.5, 1.0, 135})
        ->with_bot ({ourRobots.at(2), true}, {2.2, -1.8, 90})
        ->with_bot ({0, false},              {3.2, 1.3, 315})
        ->with_bot ({1, false},              {2.0, -2.0, 90})
        ->with_ball({3.29, 1.24})
        ->stationary_ball()
        ->build()
    );
}

Result FreeKickTest::check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom) {
    tactic.Update();
    return Result::RUNNING;
}

void FreeKickTest::beforeTest(roboteam_msgs::World const & world) {
    testStart = now();
    ::rtt::RobotDealer::initialize_robots(0, {1, 2, 3, 4, 5});
    //awaitWorld();
    while (ros::ok() && !tactic.isInitialized) {
        ros::Duration(.2).sleep();
        ros::spinOnce();
        tactic.Initialize();
    }
}

std::string FreeKickTest::testName() {
    return "FreeKickTest";
}
    
}
}