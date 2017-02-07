#include <memory>
#include <ros/ros.h>

#include "roboteam_utils/constants.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/practice_tests/PracticeTest.h"
#include "roboteam_tactics/practice_tests/Side.h"
#include "roboteam_tactics/practice_tests/KeeperTest.h"
#include "roboteam_tactics/practice_tests/FreeKickTest.h"
#include "roboteam_tactics/practice_tests/TestSetup.h"

#include "roboteam_utils/TeamRobot.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_utils/grSim_Packet.pb.h"
#include "roboteam_utils/grSim_Commands.pb.h"
#include "roboteam_utils/grSim_Replacement.pb.h"

#include <QtNetwork>

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
    rtt::LastWorld::set(*newWorld);
}

void callback_geom_data(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    fieldGeom = geometry->field;
    receivedGeom = true;
}

std::string const redText = "\e[38;2;255;0;0m";
std::string const greenText = "\e[38;2;0;255;0m";
std::string const yellowText = "\e[38;2;255;255;0m";
std::string const resetText = "\e[0m";

} // anonymous namespace

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "PracticeTest", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    auto worldSubscriber = n.subscribe(rtt::TOPIC_WORLD_STATE, 1, callback_world_state);
    auto geomSubscriber = n.subscribe(rtt::TOPIC_GEOMETRY, 1, callback_geom_data);

    ros::Rate rate(60);
    rtt::GlobalPublisher<roboteam_msgs::RoleDirective> globalRoleDirectivePublisher(rtt::TOPIC_ROLE_DIRECTIVE);
    auto directivePub = globalRoleDirectivePublisher.get_publisher();

    int const numNodes = 2;
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

    std::unique_ptr<PracticeTest> keeperTest(new FreeKickTest());

    std::vector<::rtt::RobotID> robots = {0, 1, 2, 3, 4, 5};
    std::random_device rd;
    std::mt19937 g(rd());
    // std::shuffle(robots.begin(), robots.end(), g);

    b::optional<Config> confOpt = keeperTest->getConfig(Side::RIGHT, robots, fieldGeom);

    if (confOpt) {
        RTT_DEBUGLN("Running test %s", keeperTest->testName().c_str());
    } else {
        RTT_DEBUGLN("Cannot run test %s", keeperTest->testName().c_str());
        return 1;
    }

    auto conf = *confOpt;

    // Create UDP socket
    QUdpSocket sock;

    // Reset the entire game
    send_setup_to_grsim(SetupBuilder::reset());

    // Place robots at correct positions
    // Place ball at correct position
    send_setup_to_grsim(conf);

    // Call beforeTest, wait half a second
    {
        ros::Rate fps2(2);
        fps2.sleep();
    }
    keeperTest->beforeTest(world);

    // Send roledirectives
    for (auto robot : conf.us) {
        if (robot.second.directive) {
            RTT_DEBUGLN("Sending directive...");
            auto directive = *robot.second.directive;
            directivePub.publish(*robot.second.directive);
        }
    }

    // Set speeds for robot & ball
    send_setup_to_grsim(conf);

    // Loop on check at 60fps
    ros::Rate fps(60);
    Result res = Result::RUNNING;;
    bool noAbort = true;
    do {
        fps.sleep();
        ros::spinOnce();   
        res = keeperTest->check(world, Side::LEFT, fieldGeom);

        if (!ros::ok()) {
            noAbort = false;
            break;
        }
    } while (res == Result::RUNNING);
    
    if (!noAbort) {
        std::cout << yellowText << "Test aborted!\n";
    } else if (res == Result::SUCCESS) {
        std::cout << greenText << "Test succeeded!\n";
    } else {
        std::cout << redText << "Test failed!\n";
    }

    std::cout << resetText;

    // Clean up!
    keeperTest->afterTest(world);

    send_setup_to_grsim(SetupBuilder::reset());

    return 0;
}
