#include <gtest/gtest.h>
#include "roboteam_tactics/skills/ShootAtGoalV2.h"
#include <iostream>

namespace rtt {
using namespace roboteam_msgs;

TEST(ShootAtGoalTests, SimplePartitionTest) {
    World world;
    WorldRobot shooter, keeper, extra;
    shooter.pos.x = 3.0;
    shooter.pos.y = 1.2;
    keeper.pos.x =  4.0;
    keeper.pos.y =  0.0;
    extra.pos.x =   3.7;
    extra.pos.y =   0.6;
    world.us.push_back(shooter);
    world.them.push_back(keeper);
    world.them.push_back(extra);
    world.ball.pos.x = 3.1;
    world.ball.pos.y = 0.0;
    
    GoalPartition partition(false);
    partition.calculatePartition(world, Vector2(shooter.pos));
    std::vector<Section> robotSections = partition.getRobots();
    std::cout << "Robots:\n\n";
    for (const auto& sec : robotSections) {
        std::cout << "From: " << sec.a.x << ", " << sec.a.y << " || To: " << sec.b.x << ", " << sec.b.y << "\n";
    }
    std::cout << "\nBlocked:\n\n";
    for (const auto& sec : partition.getBlocked()) {
        std::cout << "From: " << sec.a.x << ", " << sec.a.y << " || To: " << sec.b.x << ", " << sec.b.y << "\n";
    }
    std::cout << "\nOpen:\n\n";
    for (const auto& sec : partition.getOpen()) {
        std::cout << "From: " << sec.a.x << ", " << sec.a.y << " || To: " << sec.b.x << ", " << sec.b.y << "\n";
    }
    
    auto largest = partition.largestOpenSection();
    std::cout << "\nLargest: " << largest->a.x << ", " << largest->a.y << 
                " || To: " << largest->b.x << ", " << largest->b.y << "\n";
}
    
}