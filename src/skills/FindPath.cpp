#include <chrono>
#include <fstream>
#include <ros/ros.h>
#include <memory>
#include <boost/optional.hpp>
#include <tuple>

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

namespace {

rtt::Pos convertPosition(rtt::Vector2 const & pos) {
    auto geom = rtt::LastWorld::get_field();

    return {
        (int) ((pos.x + 0.5 * geom.field_length) / geom.field_length * 900),
        (int) ((pos.y + 0.5 * geom.field_width) / geom.field_width * 600)
    };
}

rtt::Vector2 convertPosition(rtt::Pos const & pos) {
    auto geom = rtt::LastWorld::get_field();

    return {
        ((pos.x / 900.0) - 0.5) * geom.field_length,
        ((pos.y / 600.0) - 0.5) * geom.field_width
    };
}

rtt::Pos convertPosition(roboteam_msgs::Vector2f const & pos) {
    return convertPosition(rtt::Vector2(pos));
}

void debugToFile(rtt::Grid<900, 600> const & grid, std::vector<rtt::Pos> const & currentPathPos, std::set<rtt::Pos> visited) {
    std::ofstream gridFile("/home/bobe/garbage/grid_test.txt");
    int const gridSize = std::pow(10, std::floor(std::log(std::min(600, 900))/std::log(10)));

    gridFile << grid.toString(gridSize);

    gridFile << "\n\n\n\n\n\n";

    gridFile << grid.toString(gridSize, currentPathPos, visited);
}

}

namespace rtt {

RTT_REGISTER_SKILL(FindPath);

FindPath::FindPath(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , lastPathSearch{}
        , gtpBb(std::make_shared<bt::Blackboard>())
        , gtp("", gtpBb)
        { 
    std::cout << "Construction FindPath..." << std::endl;

    // gtpBb = std::make_shared<bt::Blackboard>();
    // gtp.SetBlackboard(gtpBb);
}

void FindPath::Initialize() {
    // std::cout << "Initializing FindPath..." << std::endl;

    // for (int i = 0; i < 20; i++) {
        // std::cout << "Findpath: " << i << "\n";
        // findPath();
    // }
}

void FindPath::findPath() {
    unsigned int const ROBOT_ID = GetInt("ROBOT_ID");
    double const xGoal = GetDouble("xGoal");
    double const yGoal = GetDouble("yGoal");

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
    // Normally the robot has a diameter of 18 cm, so radius of 9. Also 2 cm padding, so 1 cm extra
    int robotRadius = 9 * scalar;

    auto world = LastWorld::get();

    for (auto const & robot : world.them) {
        grid.placeRobot(convertPosition(robot.pos), robotRadius * 2);
    }

    for (auto const & robot : world.us) {
        if (robot.id != ROBOT_ID) grid.placeRobot(convertPosition(robot.pos), robotRadius * 2);
    }

    auto currentRobotOpt = getWorldBot(ROBOT_ID, true, world);

    if (!currentRobotOpt) {
        ROS_ERROR("Robot with ID %d could not be found!", ROBOT_ID);
        return;
    }

    auto currentRobot = *currentRobotOpt;

    using namespace std::chrono;

    using Clock = steady_clock;

    auto start = Clock::now();

    pointIndex = 0;
    // Set stack to 32 megabytes. Eeuw.
    setStackSize(32 * 1024 * 1024);
    auto pac = aStar<diagonalDist, PriorityBucketQueue, false, true, 900, 600, 7>(
                grid, 
                convertPosition(currentRobot.pos),
                convertPosition(Vector2(xGoal, yGoal))
                );
    auto currentPathPos = std::get<0>(pac);
    auto visitedSet = std::get<1>(pac);

    auto end = Clock::now();

    std::cout << "Duration of path search: " << duration_cast<milliseconds>(end - start).count() << "ms\n";

    for (auto const & pos : currentPathPos) {
        currentPath.push_back(convertPosition(pos));
    }

    if (currentPath.size() > 0) {
        auto prev = currentPath[0];
        draw.drawPoint("pathfind_point_0", prev);

        for (size_t i = 1; i < currentPath.size(); ++i) {
            auto next = currentPath[i];
            // draw.drawPoint("pathfind_point_" + std::to_string(i), pos);
            draw.drawLineAbs("pathfind_line_" + std::to_string(i), prev, next);
            prev = next;
        }
    }

    // if (!wroteToFile && currentPath.size() == 0) {
        // debugToFile(grid, currentPathPos, visitedSet);
        // wroteToFile = true;
    // }

    // std::cout << "Finished finding path!\n";
    // std::cout << "Path size: " << currentPath.size() << "\n";

    if (gtp.IsRunning()) {
        gtp.Terminate(Status::Running);
    }
    gtp.Initialize();

    lastPathSearch = Clock::now();
}

bt::Node::Status FindPath::vanillaGoToPos() {
    return runGoToPos({GetDouble("xGoal"), GetDouble("yGoal")});
}

bt::Node::Status FindPath::runGoToPos(Vector2 pos) {
    gtpBb->SetInt("ROBOT_ID", GetInt("ROBOT_ID"));
    gtpBb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
    
    gtpBb->SetDouble("xGoal", pos.x);
    gtpBb->SetDouble("yGoal", pos.y);

    return gtp.Update();
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

    while (pointIndex < currentPath.size()) {
        auto const & currentWp = currentPath[pointIndex];
        if ((currentWp - currentPos).length() <= WP_THRESHOLD) {
            pointIndex++;
        } else {
            // We found the next point!
            break;
        }
    }


    if (pointIndex < currentPath.size()) {
        if (pointIndex == currentPath.size() - 1) {
            draw.drawPoint("pathfind_current_point_robot_" + std::to_string(GetInt("ROBOT_ID")), currentPath[pointIndex]);
            return runGoToPos(currentPath[pointIndex]);
        } else {
            auto targetPos = currentPos + (currentPath[pointIndex] - currentPos).normalize() * 0.5;
            draw.drawPoint("pathfind_current_point_robot_" + std::to_string(GetInt("ROBOT_ID")), targetPos);
            return runGoToPos(targetPos);
        }
    } else {
        return vanillaGoToPos();
    }
}

bt::Node::Status FindPath::Update() {
    if ((Clock::now() - lastPathSearch) > std::chrono::milliseconds(20)) {
        findPath();
    }

    std::cout << "Path size: " << currentPath.size() << "\n";

    if (currentPath.size() == 0 || pointIndex >= currentPath.size()) {
        std::cout << "ENDING!\n";
        return vanillaGoToPos();   
    } else {
        return followCurrentPath();
    }
}

} // rtt
