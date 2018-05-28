#pragma once

#include <boost/optional.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <iostream>

#include <ros/message_forward.h>

namespace roboteam_msgs {

ROS_DECLARE_MESSAGE(RobotCommand);
ROS_DECLARE_MESSAGE(WorldRobot);
ROS_DECLARE_MESSAGE(WorldBall);
ROS_DECLARE_MESSAGE(World);

} // roboteam_msgs


#include "roboteam_utils/constants.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/TeamRobot.h"
#include "roboteam_utils/Cone.h"
#include "roboteam_msgs/World.h"

#include "roboteam_tactics/bt.hpp"

namespace roboteam_utils {

// Forward declare s.t. we don't have to include it
class Vector2;

} // roboteam_utils

namespace rtt {

/**
 * \brief Deletes an item from a vector.
 * \tparam T The type stored in the vector. operator== must be defined on T.
 */
template<typename T>
void delete_from_vector(std::vector<T> &items, const T &item) {
    auto it = std::find(items.begin(), items.end(), item);
    if (it != items.end()) {
        items.erase(it);
    }
}

/**
 * \brief Gets the names of all active ROS nodes which have subscribed to a given topic
 */
std::vector<std::string> getNodesSubscribedTo(std::string topic);

/**
 * \brief Gets the ROS namespace the caller lives in
 */
std::string getMyNamespace();

/**
 * \brief Calculates the target angle for GetBall. See its documentation for details.
 */
double GetTargetAngle(Vector2 startPos, std::string target, int theirID, bool target_our_team);
Vector2 getTargetPos(std::string target, int theirID, bool target_our_team);

/**
 * \brief Gets the robot which currently holds the ball, if one does.
 * \return A pair where the left element is a robot, and the right element is true if the robot is in our team.
 */
boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>> getBallHolder();

/**
 * \brief Gets all robots on a line between a robot and a point.
 * \param bot The robot to start the line at
 * \param point The point to end the line at
 * \param world_ptr The world where the robots exist
 * \param sight_only False if the width of the robot should be taken into account (for moving)
 * \return A vector of robots which are in the way
 */
std::vector<roboteam_msgs::WorldRobot> getObstacles(const roboteam_msgs::WorldRobot& bot,
                                                    const Vector2& point,
                                                    const roboteam_msgs::World* world_ptr = nullptr,
                                                    bool sight_only = false,
													bool ignore_both_ends = false);
                                                    
/**
 * \brief Gets all robots on a line between two points
 * \param bot The point to start the line at
 * \param point The point to end the line at
 * \param world_ptr The world where the robots exist
 * \param sight_only False if the width of the robot should be taken into account (for moving)
 * \return A vector of robots which are in the way
 */
std::vector<roboteam_msgs::WorldRobot> getObstacles(const Vector2& bot_pos,
                                                    const Vector2& point,
                                                    const roboteam_msgs::World* world_ptr = nullptr,
                                                    bool sight_only = false,
													bool ignore_both_ends = false);

/**
 * \brief Gets all robots within a conical area
 * \param world The world where the robots exist
 * \param cone The Cone describing the area to search
 * \return A vector of all robots which are at least partially within the cone.
 */                                                    
std::vector<roboteam_msgs::WorldRobot> getRobotsInCone(const roboteam_msgs::World& world, const Cone& cone);


/**
 * \brief Predict the ball position in the future based on the current velocity
 */
Vector2 predictBallPos(double seconds);

/**
 * \brief Predict a robot's position in the future based on the current velocity
 */
Vector2 predictRobotPos(uint robot_id, bool our_team, double seconds);

/**
 * \brief Prints a textual representation of the contents of a blackboard to an output stream.
 */
void print_blackboard(const bt::Blackboard::Ptr bb, std::ostream& out = std::cout);

/**
 * \brief Copies the contents of the second blackboard into the first, overwriting any elements
 * which already existed in the target.
 */
void merge_blackboards(bt::Blackboard::Ptr target, const bt::Blackboard::Ptr extras);

/**
 * \brief Gets a random integer in the range [0, max).
 */
int get_rand_int(int max);

/**
 * \brief Gets a random integer in the range [min, max).
 */
int get_rand_int(int min, int max);

/**
 * \brief Gets a random real number in the range [0, max).
 */
float get_rand_real(float max);

/**
 * \brief Gets a random real number in the range [min, max).
 */
float get_rand_real(float min, float max);

/**
 * \brief Gets a random element from a container
 * \tparam T The type of element to return
 * \tparam Container The type of the container, which must adhere to the Container C++ concept
 * \return An optional containing a random element of the container if one exists, or an empty
 * optional if it is empty
 */
template<typename T, typename Container>
boost::optional<T> get_rand_element(const Container&);

/**
 * \brief Gets a random element from a sequence
 * \tparam T The type of element to return
 * \tparam Sequence The type of the sequence, which must adhere to the Sequence C++ concept
 * \return An optional containing a random element of the sequence if one exists, or an empty
 * optional if it is empty
 */
template<typename T, typename Sequence>
boost::optional<T> get_rand_element_seq(const Sequence&);

/**
 * \brief Tests whether or not a sequene contains a specific element
 * \tparam T The type of the element to search for
 * \tparam Sequence The type of the sequence, which must adhere to the Sequence C++ concept
 */
template<typename T, typename Sequence>
bool sequence_contains(const Sequence& seq, T element);

using time_point = std::chrono::steady_clock::time_point;
using seconds = std::chrono::seconds;
using milliseconds = std::chrono::milliseconds;

/**
 * \brief Gets the current time
 */
time_point now();

/**
 * \brief Calculates the difference between to time_points in seconds
 */
seconds time_difference_seconds(time_point start, time_point end);

/**
 * \brief Calculates the difference between to time_points in milliseconds
 */
milliseconds time_difference_milliseconds(time_point start, time_point end);

/**
 * \brief Returns a string with the name of a Status enum constant
 */
std::string describe_status(bt::Node::Status status);

/**
 * \brief Returns either "left" or "right", depending on which side is ours
 */
std::string get_our_side();

/**
 * \brief Constructs a stop command for a robot with the given id
 */
roboteam_msgs::RobotCommand stop_command(unsigned int id);

/**
 * \brief Tests whether all characters in a string are digits
 */
bool is_digits(const std::string &str);

/**
 * \brief Gets the id of whichever robot is closest to a given point
 * \param robots The robots to consider
 * \param world The world in which the robots exist
 * \param point The point to search near
 * \return The id of the robot closest to the point
 */
boost::optional<int> get_robot_closest_to_point(std::vector<int> robots, const roboteam_msgs::World& world, const Vector2& point);

/**
 * \brief Gets the id of whichever robot is closest to a given point
 * \param robots The robots to consider
 * \param point The point to search near
 * \return The id of the robot closest to the point
 */
boost::optional<int> get_robot_closest_to_point(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point);

/**
 * \brief Gets the id of whichever robot is in the future closest to a given point
 * \param robots The robots to consider
 * \param point The point to search near
 * \param t_ahead The amount of seconds to look into the future
 * \return The id of the robot closest to the point
 */
boost::optional<int> predict_robot_closest_to_point(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point, double t_ahead);

/**
 * \brief Gets the id of whichever robot is closest to the opponents' goal
 * \param robots The robots to consider
 */
boost::optional<int> get_robot_closest_to_their_goal(std::vector<int> robots);


/**
 * \brief Gets the id of whichever robot is closest to our goal
 * \param robots The robots to consider
 */
// int get_robot_closest_to_our_goal(std::vector<roboteam_msgs::WorldRobot> robots);
boost::optional<int> get_robot_closest_to_our_goal(std::vector<int> robots);


/**
 * \brief Gets the id of whichever robot is closest to the ball
 * \param robots The robots to consider
 */
boost::optional<int> get_robot_closest_to_ball(std::vector<int> robots);

/**
 * \brief Gets the id of whichever robot is closest to the opponents' goal
 * \param robots The robots to consider
 * \param world The world in which the robots exist
 * \return The id of the robot closest to the goal
 */
boost::optional<int> get_robot_closest_to_their_goal(std::vector<int> robots, const roboteam_msgs::World &world);

/**
 * \brief Gets the id of whichever robot is closest to the ball
 * \param robots The robots to consider
 * \param world The world in which the robots exist
 * \return The id of the robot closest to the ball
 */
boost::optional<int> get_robot_closest_to_ball(std::vector<int> robots, const roboteam_msgs::World &world);

/**
 * \brief Tests whether a robot is within the bounds of the field. That is:
 *    -4.5 <= X <= 4.5
 * and
 *    -3.0 <= Y <= 3.0
 */
bool robotIsWithinBounds(const TeamRobot& bot, const roboteam_msgs::World& world);

boost::optional<roboteam_msgs::WorldRobot> getBotFromDangerList(unsigned dangerIndex);

bool isWithinField(Vector2 point);

/**
 * \class GlobalPublisher
 * \brief Shared ros::Publisher instance of all messages of a certain type
 * \tparam M The message type
 */
template<
    class M
>
class GlobalPublisher {
public:
    /**
     * \brief Constructor.
     * \param topic The topic on which to publish
     * \param queue The maximum queue size for messages
     */
    GlobalPublisher(std::string topic, int queue = 100)
        : actualPub(n.advertise<M>(topic, queue)) {
        if (!GlobalPublisher<M>::pubPtr) {
            GlobalPublisher<M>::pubPtr = &actualPub;
        }
    }

    /**
     * \brief Gets the shared publisher
     */
    static ros::Publisher& get_publisher() {
        if (!GlobalPublisher<M>::pubPtr) {
            ROS_ERROR("Publisher requested while it was not yet initialized!");
        }

        return *GlobalPublisher<M>::pubPtr;
    }

private:
    ros::NodeHandle n;
    ros::Publisher actualPub;
    static ros::Publisher *pubPtr;

} ;

template<
    class M
>
ros::Publisher* GlobalPublisher<M>::pubPtr = nullptr;





int getClosestOppToPoint(Vector2 testPosition, roboteam_msgs::World world);








/**
 * \brief Get all robots active in the world. This is the union of world.us and world.them
 */
std::vector<roboteam_msgs::WorldRobot> getAllBots(const roboteam_msgs::World& world = LastWorld::get());

/**
 * \brief Gets all robots active in the world as TeamRobot structures
 */
std::vector<TeamRobot> getAllTeamBots(const roboteam_msgs::World& world = LastWorld::get());

/**
 * \brief Gets the robot with the given ID as a WorldRobot
 * @param id The ID of the desired robot
 * @param ourTeam whether or not the robot is in our team
 * @param world The world to search
 * @return An optional containing the robot, or an empty optional if it was not found
 */
boost::optional<roboteam_msgs::WorldRobot> getWorldBot(unsigned int id, bool ourTeam = true, const roboteam_msgs::World& world = LastWorld::get());

/**
 * \brief Gets the robot with the given ID as a TeamRobot
 * @param id The ID of the desired robot
 * @param ourTeam whether or not the robot is in our team
 * @param world The world to search
 * @return An optional containing the robot, or an empty optional if it was not found
 */
boost::optional<TeamRobot> getTeamBot(unsigned int id, bool ourTeam = true, const roboteam_msgs::World& world = LastWorld::get());

/**
 * \brief Checks whether the parameter is the NaN value.
 */
constexpr bool isNaN(double	t) {
	return t != t;
}

bool weAreLeft();

double calcDistGoalToRobot(int robot, roboteam_msgs::World world);

std::vector<int> getAvailableRobots(roboteam_msgs::World const & world = LastWorld::get());

} // rtt

#include "roboteam_tactics/utils/utils.tcc"
