#include "roboteam_tactics/treegen/LeafRegister.h"
#include <chrono>
#include <fstream>

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/FindPath.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_tactics/paths/utils.h"
#include "roboteam_tactics/paths/Grid.h"
#include "roboteam_tactics/paths/AStar.h"

namespace rtt {

RTT_REGISTER_SKILL(FindPath);

FindPath::FindPath(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) { }

void FindPath::Initialize() {
    findPath();

    start = Clock::now();
}

Pos FindPath::convertPosition(Vector2 pos) {
    std::cout << "Converting " << pos << "\n";

    auto geom = LastWorld::get_field();

    return {
        (int) ((pos.x + 0.5 * geom.field_length) / geom.field_length * 900),
        (int) ((pos.y + 0.5 * geom.field_width) / geom.field_width * 600)
    };
}

Pos FindPath::convertPosition(roboteam_msgs::Vector2f pos) {
    return convertPosition(Vector2(pos));
}

void FindPath::findPath() {
    int const ROBOT_ID = GetInt("ROBOT_ID");
    double const xGoal = GetDouble("xGoal");
    double const yGoal = GetDouble("yGoal");

    std::cout << "Finding path...\n";

    currentPath.clear();

    Grid<900, 600> grid;

    auto geom = LastWorld::get_field();
    double realWidth = geom.field_width;
    double internalWidth = 6;

    double scalar = internalWidth / realWidth;
    // Normally the robot has a diameter of 18 cm, so radius of 9
    int robotRadius = 9 * scalar;
    std::cout << "RobotRadius: " << robotRadius << "\n";

    auto world = LastWorld::get();

    std::cout << "Num robots: " << world.them.size() + world.us.size() << "\n";

    for (auto const & robot : world.them) {
        grid.placeRobot(convertPosition(robot.pos), robotRadius);
    }

    for (auto const & robot : world.us) {
        if (robot.id != ROBOT_ID) grid.placeRobot(convertPosition(robot.pos), robotRadius);
    }

    auto currentRobotOpt = getWorldBot(ROBOT_ID, true, world);

    if (!currentRobotOpt) {
        ROS_ERROR("Robot with ID %d could not be found!", ROBOT_ID);
        return;
    }

    auto currentRobot = *currentRobotOpt;

    std::ofstream gridFile("/home/bobe/garbage/grid_test.txt");
    int const gridSize = std::pow(10, std::floor(std::log(std::min(600, 900))/std::log(10)));
    gridFile << grid.toString(gridSize);

    currentPath = std::get<0>(aStar<diagonalDist, PriorityBucketQueue, false, false, 900, 600, 7>(grid, convertPosition(currentRobot.pos), convertPosition(Vector2(xGoal, yGoal))));

    gridFile << "\n\n\n\n\n\n";
    gridFile << grid.toString(gridSize, currentPath);

    std::cout << "Finished finding path!\n";
    std::cout << "Path size: " << currentPath.size() << "\n";
}

bt::Node::Status FindPath::Update() {


    return Status::Running;
}

} // rtt
