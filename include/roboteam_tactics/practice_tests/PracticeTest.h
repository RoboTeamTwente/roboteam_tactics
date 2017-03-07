#pragma once

#include <vector>
#include <map>

#include <boost/optional.hpp>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/TeamRobot.h"
#include "roboteam_tactics/treegen/LeafRegister.h"

namespace rtt {

namespace practice {

using ::Vector2;
using ::Position;

struct Robot {
    Position pos;
    Position speed;

    boost::optional<roboteam_msgs::RoleDirective> directive;
} ;

struct Config {
    Vector2 ballPos;
    Vector2 ballSpeed;

    std::map<RobotID, Robot> us;
    std::map<RobotID, Robot> them;
};

enum class Result {
    RUNNING,
    FAILURE,
    SUCCESS
} ;

enum class Side;

class PracticeTest {
public:
    virtual ~PracticeTest();

    virtual boost::optional<Config> getConfig(
            Side side, 
            std::vector<RobotID> const & ourRobots, 
            roboteam_msgs::GeometryFieldSize const & fieldGeom
            );

    virtual void beforeTest(roboteam_msgs::World const & world);
    virtual Result check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom);
    virtual void afterTest(roboteam_msgs::World const & world);

    virtual std::string testName() = 0;
    
    static bool awaitWorld();
} ;

} // namespace practice

} // namespace rtt
