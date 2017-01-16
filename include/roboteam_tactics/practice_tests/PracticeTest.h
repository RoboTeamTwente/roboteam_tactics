#pragma once

#include <vector>
#include <map>

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

    roboteam_msgs::RoleDirective directive;
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

class PracticeTest {
public:
    virtual ~PracticeTest();

    virtual Config getConfig();

    virtual void beforeTest(roboteam_msgs::World const & world);
    virtual Result check(roboteam_msgs::World const & world);
    virtual void afterTest(roboteam_msgs::World const & world);
} ;

} // namespace practice

} // namespace rtt
