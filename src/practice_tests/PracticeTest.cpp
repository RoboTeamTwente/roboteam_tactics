#include "roboteam_tactics/practice_tests/PracticeTest.h"

namespace rtt {

namespace practice {

namespace b = boost;

PracticeTest::~PracticeTest() {

}

b::optional<Config> PracticeTest::getConfig(
    Side side, 
    std::vector<int> ourRobots, 
    roboteam_msgs::GeometryFieldSize fieldGeom
    ) {
    return b::none;
}

void PracticeTest::beforeTest(roboteam_msgs::World const & world) {

}

Result PracticeTest::check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom) {
    
}

void PracticeTest::afterTest(roboteam_msgs::World const & world) {

}

} // namespace practice

} // namespace rtt
