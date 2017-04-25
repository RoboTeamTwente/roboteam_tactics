#include <gtest/gtest.h>
#include "roboteam_tactics/skills/ShootAtGoalV2.h"
#include <iostream>

namespace rtt {
using namespace roboteam_msgs;

template<typename T>
void testVector(std::vector<T> expected, const std::vector<T>& actual, std::string name = "") {
	ASSERT_EQ(expected.size(), actual.size()) << "(" << name << ")\n";
	for (size_t i = 0; i < actual.size(); i++) {
		ASSERT_EQ(expected.at(i), actual.at(i)) << name << " vector differs at " << i << "\n";
	}
}

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
    auto largest = partition.largestOpenSection();
    testVector({Section(4.06914,0.0576166,3.93086,-0.0576166),
    		    Section(3.75857,0.668333,3.64143,0.531667)}, robotSections, "Robot");

    testVector({Section(4.5, 0.148681, 4.5, -0.362916),
    			Section(4.5, -0.40276, 4.5, -0.5)}, partition.getBlocked(), "Blocked");

    testVector({Section(4.5, 0.5, 4.5, 0.148681),
    			Section(4.5, -0.362916, 4.5, -0.40276)}, partition.getOpen(), "Open");
    /*
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
    
    std::cout << "\nLargest: " << largest->a.x << ", " << largest->a.y << 
                " || To: " << largest->b.x << ", " << largest->b.y << "\n";
    */
}
    
}
