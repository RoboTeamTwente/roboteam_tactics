#include <memory>
#include <ros/ros.h>

#include "roboteam_utils/constants.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/practice_tests/PracticeTest.h"
#include "roboteam_tactics/practice_tests/Side.h"
#include "roboteam_tactics/practice_tests/KeeperTest.h"

namespace b = boost;
using namespace rtt::practice;

#define RTT_CURRENT_DEBUG_TAG PracticeTest

namespace {

bool receivedWorld = false;
bool receivedGeom = false;

roboteam_msgs::World world;
roboteam_msgs::GeometryFieldSize fieldGeom;

void callback_world_state(const roboteam_msgs::WorldConstPtr& newWorld) {
    world = *newWorld;
    receivedWorld = true;
}

void callback_geom_data(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    fieldGeom = geometry->field;
    receivedGeom = true;
}

} // anonymous namespace

int main(int argc, char *argv[]) {
    using rtt::practice::KeeperTest;
    using rtt::practice::PracticeTest;

    ros::NodeHandle n;

    auto worldSubscriber = n.subscribe(rtt::TOPIC_WORLD_STATE, 1, callback_world_state);
    auto geomSubscriber = n.subscribe(rtt::TOPIC_GEOMETRY, 1, callback_geom_data);

    ros::Rate rate(60);
    auto directivePub = n.advertise<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE, 100);

    int const numNodes = 6;
    RTT_DEBUGLN("Waiting for %i robot nodes to come online", numNodes);
   
    while ((int) directivePub.getNumSubscribers() < numNodes) {
        ros::spinOnce();
        rate.sleep();

        if (!ros::ok()) {
            RTT_DEBUGLN("Interrupt received, exiting...");
            return 0;
        }
    }

    RTT_DEBUGLN("Waiting for the first world & geometry messages...");

    while (!(receivedWorld && receivedGeom)) {
        ros::spinOnce();
        rate.sleep();

        if (!ros::ok()) {
            RTT_DEBUGLN("Interrupt received, exiting...");
        }
    }

    std::unique_ptr<PracticeTest> keeperTest(new KeeperTest());

    std::vector<int> robots = {0, 1, 2, 3, 4, 5};
    b::optional<Config> conf = keeperTest->getConfig(Side::RIGHT, robots, fieldGeom);

    if (conf) {
        RTT_DEBUGLN("Running test %s\n", keeperTest->testName().c_str());
    } else {
        RTT_DEBUGLN("Cannot run test %s\n", keeperTest->testName().c_str());
    }

    return 0;
}
