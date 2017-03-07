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

/**
 * \struct Robot
 * \brief Stores information regarding a robot relevant to Practice Tests
 */
struct Robot {
    Position pos;   /**< The robot's position */
    Position speed; /**< The robot's velocity */

    boost::optional<roboteam_msgs::RoleDirective> directive; /**< An optional RoleDirective for the robot */
} ;

/**
 * \struct Config
 * \brief A test configuration
 */
struct Config {
    Vector2 ballPos;                /**< The initial position of the ball */
    Vector2 ballSpeed;              /**< The initial velocity of the ball */

    std::map<RobotID, Robot> us;    /**< Configurations for our robots */
    std::map<RobotID, Robot> them;  /**< Configurations for the opponents */
};

/**
 * \enum Result
 * \brief Practice test results
 */
enum class Result {
    RUNNING,
    FAILURE,
    SUCCESS
} ;

enum class Side;

/**
 * \class PracticeTest
 * \brief A static setup to test a specific situaion.
 */
class PracticeTest {
public:
    virtual ~PracticeTest();

    /**
     * \brief Get the Config which should be used to prepare the test
     * \param side The side to test on
     * \param ourRobots The participating robots
     * \param fieldGeom The field geometry
     * \return An optional containing the Config if one can be created with these parameters, or an empty optional otherwise.
     */
    virtual boost::optional<Config> getConfig(
            Side side, 
            std::vector<RobotID> const & ourRobots, 
            roboteam_msgs::GeometryFieldSize const & fieldGeom
            );

    /**
     * \brief Gets everything ready for the test, with the exception of applying the Config.
     * \param world The world we are testing in.
     */
    virtual void beforeTest(roboteam_msgs::World const & world);
    
    /**
     * \brief Performs a single step of the test and returns the current result.
     * \param world The world we are testing in
     * \param side The side to test on
     * \param fieldGeom The field geometry
     * \return The (intermediate) result after performing a tick
     */
    virtual Result check(roboteam_msgs::World const & world, Side side, roboteam_msgs::GeometryFieldSize const & fieldGeom);
    
    /**
     * \brief Cleanup to be executed after the test.
     * \param world The world we were testing in
     */
    virtual void afterTest(roboteam_msgs::World const & world);

    /**
     * \return A unique name to identify this test by 
     */
    virtual std::string testName() = 0;
    
    /**
     * \brief Wait for the world to be ready for the test. This method will wait until some robots exist.
     * \return Whether, after waiting, the test can proceed
     */
    static bool awaitWorld();
} ;

} // namespace practice

} // namespace rtt
