#include <chrono>
#include <fstream>
#include <ros/ros.h>
#include <memory>
#include <boost/optional.hpp>

#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/FindPath.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_tactics/paths/utils.h"
#include "roboteam_tactics/paths/Grid.h"
#include "roboteam_tactics/paths/AStar.h"

namespace b = boost;

namespace rtt {

RTT_REGISTER_SKILL(FindPath);

FindPath::FindPath(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , gtpBb(std::make_shared<bt::Blackboard>())
        , gtp("", gtpBb)
        { 
    std::cout << "Construction FindPath..." << std::endl;

    // gtpBb = std::make_shared<bt::Blackboard>();
    // gtp.SetBlackboard(gtpBb);
}

void FindPath::Initialize() {
    std::cout << "Initializing FindPath..." << std::endl;

    findPath();
}

Pos FindPath::convertPosition(Vector2 pos) {
    auto geom = LastWorld::get_field();

    return {
        (int) ((pos.x + 0.5 * geom.field_length) / geom.field_length * 900),
        (int) ((pos.y + 0.5 * geom.field_width) / geom.field_width * 600)
    };
}

Vector2 FindPath::convertPosition(Pos pos) {
    auto geom = LastWorld::get_field();

    return {
        ((pos.x / 900.0) - 0.5) * geom.field_length,
        ((pos.y / 600.0) - 0.5) * geom.field_width
    };
}

Pos FindPath::convertPosition(roboteam_msgs::Vector2f pos) {
    return convertPosition(Vector2(pos));
}

void FindPath::findPath() {
    std::cout << "Finding path..." << std::endl;

    unsigned int const ROBOT_ID = GetInt("ROBOT_ID");
    double const xGoal = GetDouble("xGoal");
    double const yGoal = GetDouble("yGoal");

    std::cout << "Set up local vars." << std::endl;


    // TODO: Everything is so icky right now because the aStar routine
    // wants to work with a *compile-time* size of a 900x600 field.
    // Ideally this'd be flexible at runtime, but I made it fixed at
    // compile time. If there is time, refactor this. The performance
    // hit is probably acceptable.
    // TODO: Points of improvement: perfecting pathfollowing, prediction
    // of the future (objects on the field are not static, incorporate 
    // their speeds somehow), cooperative path scheduling (when two 
    // robots are crossing paths, one should give the other priority,
    // instead of bumping into eachother)
    // TODO: Can the paths utils be removed from the FindPath header?

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
        grid.placeRobot(convertPosition(robot.pos), robotRadius * 2);
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

    // std::ofstream gridFile("/home/bobe/garbage/grid_test.txt");
    // int const gridSize = std::pow(10, std::floor(std::log(std::min(600, 900))/std::log(10)));
    // gridFile << grid.toString(gridSize);

    pointIndex = 0;
    currentPath = std::get<0>(aStar<diagonalDist, PriorityBucketQueue, false, false, 900, 600, 7>(grid, convertPosition(currentRobot.pos), convertPosition(Vector2(xGoal, yGoal))));

    int i = 0;
    for (auto const & pos : currentPath) {
        draw.drawPoint(
                "pathfind_point_" + std::to_string(i),
                convertPosition(pos)
                );
        ++i;
    }

    // gridFile << "\n\n\n\n\n\n";
    // gridFile << grid.toString(gridSize, currentPath);

    std::cout << "Finished finding path!\n";
    std::cout << "Path size: " << currentPath.size() << "\n";

    gtp.Initialize();
}

bt::Node::Status FindPath::vanillaGoToPos() {
    return runGoToPos({GetDouble("xGoal"), GetDouble("yGoal")});
}

bt::Node::Status FindPath::runGoToPos(Vector2 pos) {
    std::cout << "Driving towards: " << pos << "\n";
        
    gtpBb->SetDouble("xGoal", pos.x);
    gtpBb->SetDouble("yGoal", pos.y);
    return gtp.Update();
}

b::optional<Vector2> FindPath::getWaypoint(int index) {
    if (static_cast<unsigned int>(index) < currentPath.size() && index >= 0) {
        Pos pos = currentPath[index];
        return convertPosition(pos);
    } else {
        return b::none;
    }
}

bt::Node::Status FindPath::followCurrentPath() {
    unsigned int const ROBOT_ID = GetInt("ROBOT_ID");
    double const WP_THRESHOLD = 0.3; // Meters

    roboteam_msgs::WorldRobot bot;

    if (auto botOpt = getWorldBot(ROBOT_ID, true)) {
        bot = *botOpt;
    } else {
        ROS_ERROR("Could not find my robot (%x) in World; not doing anything.", ROBOT_ID);
        return Status::Running;
    }

    Vector2 currentPos = bot.pos;

    while (static_cast<unsigned int>(pointIndex) < currentPath.size()) {
        if (auto currentWpOpt = getWaypoint(pointIndex)) {
            if ((*currentWpOpt - currentPos).length() <= WP_THRESHOLD) {
                pointIndex++;
            } else {
                // We found the next point!
                break;
            }
        }
    }

    if (auto currentWpOpt = getWaypoint(pointIndex)) {
        return runGoToPos(*currentWpOpt);
    } else {
        return vanillaGoToPos();
    }
}

bt::Node::Status FindPath::Update() {
    std::cout << " --------- Running FindPath... -----------\n";

    if (currentPath.size() == 0 || static_cast<unsigned int>(pointIndex) >= currentPath.size()) {
        return vanillaGoToPos();   
    } else {
        return followCurrentPath();
    }
}

} // rtt
