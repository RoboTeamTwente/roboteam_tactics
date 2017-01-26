#pragma once

#include <vector>
#include <map>

#include <boost/optional.hpp>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

namespace practice {

using ::roboteam_utils::Vector2;

struct Robot {
    Vector2 pos;
    Vector2 speed;
    double angle;

    boost::optional<roboteam_msgs::RoleDirective> directive;
} ;

struct Config {
    Vector2 ballPos;
    Vector2 ballSpeed;

    std::map<int, Robot> us;
    std::map<int, Robot> them;
} ;

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
            std::vector<int> ourRobots, 
            roboteam_msgs::GeometryFieldSize fieldGeom
            );

    virtual void beforeTest(roboteam_msgs::World const & world);
    virtual Result check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom);
    virtual void afterTest(roboteam_msgs::World const & world);

    virtual std::string testName() = 0;
} ;

} // namespace practice

} // namespace rtt
