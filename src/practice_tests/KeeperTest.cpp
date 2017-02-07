#include <cmath>

#include "unique_id/unique_id.h" 

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/practice_tests/KeeperTest.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/practice_tests/TestSetup.h"

using namespace rtt::practice;
namespace b = boost;

b::optional<Config> KeeperTest::getConfig(
    Side side, 
    std::vector<RobotID> const & ourRobots, 
    roboteam_msgs::GeometryFieldSize const & fieldGeom
    ) {

    if (ourRobots.size() < 1) return b::none;

    double rad = rtt::get_rand_real(0.7 * M_PI, 1.3 * M_PI);
    double const r = 1.5;
    Vector2 ballPos = Vector2(4.5 + std::cos(rad) * r, std::sin(rad) * r);
    Vector2 ballSpeed = (Vector2(4.5, 0) - ballPos).normalize() * 2;
    return SetupBuilder::builder()->with_bot({ourRobots.at(0), true}, {4.3, 0, M_PI})
                                         ->with_ball(ballPos)
                                         ->with_ball_vel(ballSpeed)
                                         ->build();
}

void KeeperTest::beforeTest(roboteam_msgs::World const & world) {
    testStart = now();
}

Result KeeperTest::check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom) {
    milliseconds passedTime = time_difference_milliseconds(testStart, now());

    Vector2 ballPos(world.ball.pos);

    // TODO: Basde on side & geom
    if (ballPos.x > 4.5) {
        return Result::FAILURE;
    }

    if (passedTime.count() > 5000) {
        return Result::SUCCESS;
    }

    return Result::RUNNING;
}

// TODO: This can possibly go in the same macro that registers the test.
std::string KeeperTest::testName() {
    return "KeeperTest";
}
