#include "roboteam_tactics/practice_tests/PracticeTest.h"
#include "roboteam_utils/LastWorld.h"
namespace rtt {

namespace practice {

namespace b = boost;

PracticeTest::~PracticeTest() {

}

b::optional<Config> PracticeTest::getConfig(
    Side side, 
    std::vector<RobotID> const & ourRobots, 
    roboteam_msgs::GeometryFieldSize const &  fieldGeom
    ) {
    return b::none;
}

void PracticeTest::beforeTest(roboteam_msgs::World const & world) {

}

Result PracticeTest::check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom) {
    return Result::SUCCESS;
}

void PracticeTest::afterTest(roboteam_msgs::World const & world) {

}

bool PracticeTest::awaitWorld() {
    auto world = LastWorld::get();
    while (ros::ok() && world.us.size() == 0 && world.them.size() == 0) {
        ros::spinOnce();
        world = LastWorld::get();
    }
    return ros::ok();
}

} // namespace practice

} // namespace rtt
