#include <gtest/gtest.h>
#include "roboteam_tactics/analysis/SpaceAnalysis.h"

namespace rtt {
using namespace roboteam_msgs;    
    
void buildBot(World& world, int id, double x, double y) {
    WorldRobot bot;
    bot.id = id;
    bot.pos.x = x;
    bot.pos.y = y;
    world.them.push_back(bot);
}    
    
TEST(SpaceAnalysisTests, testTerritory) {
    int argc = 0;
    ros::init(argc, (char**) 0, "SpaceAnalysisTest");
    World world;
    buildBot(world, 0, 1, 1);
    buildBot(world, 1, 0, 0);
    buildBot(world, 2, 2, 0);
    buildBot(world, 3, 0, 2);
    buildBot(world, 4, 2, 2);
    
    auto territories = findTerritories(world);
    TeamRobot bot0 = {0, false};
    ASSERT_NE(territories.end(), territories.find(bot0));
    Territory t0 = territories.at({0, false});
    ASSERT_EQ(bot0, t0.bot);
    ASSERT_EQ(Position(1, 1, 0), t0.botPos);
    //ASSERT_EQ(Vector2(1, 1), t0.center);
    //ASSERT_DOUBLE_EQ(1.0, t0.area);
    
    drawVoronoi(world, false);
}
    
}