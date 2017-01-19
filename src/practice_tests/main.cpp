#include <memory>
#include <ros/ros.h>

#include "roboteam_utils/constants.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/practice_tests/PracticeTest.h"
#include "roboteam_tactics/practice_tests/Side.h"
#include "roboteam_tactics/practice_tests/KeeperTest.h"

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
}

void callback_geom_data(const roboteam_msgs::GeometryDataConstPtr& geometry) {
    fieldGeom = geometry->field;
    receivedGeom = true;
}

void placeRobotsAndBall(QUdpSocket & sock, Config const & conf, bool applySpeed, bool usIsYellow) {
    // Create new packet
    grSim_Packet packet;

    // Re-place all our robots
    for (auto robot : conf.us) {
        grSim_RobotReplacement* replacement = packet.mutable_replacement()->add_robots();
        replacement->set_x(robot.second.pos.x);
        replacement->set_y(robot.second.pos.y);
        replacement->set_dir(robot.second.angle);
        replacement->set_id(robot.first);
        replacement->set_yellowteam(usIsYellow);
    }

    // Re-place all their robots
    for (auto robot : conf.them) {
        grSim_RobotReplacement* replacement = packet.mutable_replacement()->add_robots();
        replacement->set_x(robot.second.pos.x);
        replacement->set_y(robot.second.pos.y);
        replacement->set_dir(robot.second.angle);
        replacement->set_id(robot.first);
        replacement->set_yellowteam(!usIsYellow);
    }

    // Re-place the ball and apply speed if needed
    grSim_BallReplacement* ballReplacement = packet.mutable_replacement()->mutable_ball();
    ballReplacement->set_x(conf.ballPos.x);
    ballReplacement->set_y(conf.ballPos.y);
    
    if (applySpeed) {
        ballReplacement->set_vx(conf.ballSpeed.x);
        ballReplacement->set_vy(conf.ballSpeed.y);
    }

    // Convert the packet
	QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());

    // Send to IP address and port specified in grSim
    std::string grsim_ip = "127.0.0.1";
    int grsim_port = 20011;
    ros::param::get("grsim/ip", grsim_ip);
    ros::param::get("grsim/port", grsim_port);
    sock.writeDatagram(dgram, QHostAddress(QString::fromStdString(grsim_ip)), grsim_port);
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
    b::optional<Config> confOpt = keeperTest->getConfig(Side::RIGHT, robots, fieldGeom);

    if (confOpt) {
        RTT_DEBUGLN("Running test %s\n", keeperTest->testName().c_str());
    } else {
        RTT_DEBUGLN("Cannot run test %s\n", keeperTest->testName().c_str());
        return 1;
    }

    auto conf = *confOpt;

    // Create UDP socket
    QUdpSocket sock;

    // Place robots at correct positions
    // Place ball at correct position
    placeRobotsAndBall(sock, conf, false, true);

    // Call beforeTest, wait half a second
    {
        ros::Rate fps2(2);
        fps2.sleep();
    }
    keeperTest->beforeTest(world);

    // Send roledirectives
    for (auto robot : conf.us) {
        directivePub.publish(robot.second.directive);
    }

    // Set speeds for robot & ball
    placeRobotsAndBall(sock, conf, true, true);

    // Loop on check at 60fps
    ros::Rate fps(60);
    Result res = Result::RUNNING;;
    do {
        fps.sleep();
        ros::spinOnce();   
        res = keeperTest->check(world, Side::LEFT, fieldGeom);
    } while (res == Result::RUNNING);
    
    // Clean up!
    keeperTest->afterTest(world);

    return 0;
}
