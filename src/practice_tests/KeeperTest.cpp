#include <cmath>

#include "unique_id/unique_id.h" 

#include "roboteam_tactics/bt.hpp"
#include "roboteam_tactics/practice_tests/KeeperTest.h"
#include "roboteam_tactics/utils/utils.h"

using namespace rtt::practice;
namespace b = boost;

b::optional<Config> KeeperTest::getConfig(
    Side side, 
    std::vector<int> ourRobots, 
    roboteam_msgs::GeometryFieldSize fieldGeom
    ) {

    if (ourRobots.size() < 1) return b::none;

    Config conf;

    // TODO: Eigenlijk wil je het ook kant-agnostisch hebben (links/rechts)
    {
        // TODO: Calculate position & angle in front of goal based on side & fieldGeom
        
        Robot keeper;
        keeper.angle = M_PI;
        keeper.pos = Vector2(4.3, 0);

        roboteam_msgs::RoleDirective rd;
        rd.robot_id = ourRobots.at(0);
        rd.tree = "rtt_bob/BasicKeeperTree";
        // Need to save this somehow maybe?
        rd.token = unique_id::toMsg(unique_id::fromRandom());
        
        bt::Blackboard bb;
        bb.SetInt("ROBOT_ID", ourRobots.at(0));
        rd.blackboard = bb.toMsg();
        
        conf.us[ourRobots.at(0)] = keeper;
    }

    // TODO: Calculate position in front of goal somewhere based on side & fieldGeom
    conf.ballPos = Vector2(2.3, 0);
    conf.ballSpeed = Vector2(3, 0);

    return conf;
}

void KeeperTest::beforeTest(roboteam_msgs::World const & world) {
    testStart = now();
}

Result KeeperTest::check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom) {
    milliseconds passedTime = time_difference_milliseconds(testStart, now());

    Vector2 ballPos(world.ball.pos);

    // TODO: Basde on side & geom
    if (ballPos.x > 4500) {
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
