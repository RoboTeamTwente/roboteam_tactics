#pragma once

#include <boost/optional.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <iostream>

#include "roboteam_tactics/bt.hpp"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/LastWorld.h"

namespace rtt {

template<typename T>
void delete_from_vector(std::vector<T> &items, const T &item) {
    auto it = std::find(items.begin(), items.end(), item);
    if (it != items.end()) {
        items.erase(it);
    }
}

std::vector<std::string> getNodesSubscribedTo(std::string topic);
std::string getMyNamespace();
double GetTargetAngle(int myID, bool our_team, std::string target, int theirID, bool target_our_team);
boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>> getBallHolder();
std::vector<roboteam_msgs::WorldRobot> getObstacles(const roboteam_msgs::WorldRobot& bot,
                                                    const roboteam_utils::Vector2& point,
                                                    const roboteam_msgs::World* world_ptr = nullptr,
                                                    bool sight_only = false);

/**
 * Predict the ball or robot position in the future based on the current velocity
 */
roboteam_utils::Vector2 predictBallPos(double seconds);
roboteam_utils::Vector2 predictRobotPos(uint robot_id, bool our_team, double seconds);



void print_blackboard(const bt::Blackboard::Ptr bb, std::ostream& out = std::cout);
void merge_blackboards(bt::Blackboard::Ptr target, const bt::Blackboard::Ptr extras);

/**
 * Gets a random integer in the range [0, max).
 */
int get_rand_int(int max);

/**
 * Gets a random integer in the range [min, max).
 */
int get_rand_int(int min, int max);

/**
 * Gets a random real number in the range [0, max).
 */
float get_rand_real(float max);

/**
 * Gets a random real number in the range [min, max).
 */
float get_rand_real(float min, float max);

using time_point = std::chrono::steady_clock::time_point;
using seconds = std::chrono::seconds;
using milliseconds = std::chrono::milliseconds;

time_point now();

seconds time_difference_seconds(time_point start, time_point end);
milliseconds time_difference_milliseconds(time_point start, time_point end);

std::string describe_status(bt::Node::Status status);

std::string get_our_side();
roboteam_msgs::RobotCommand stop_command(unsigned int id);

bool is_digits(const std::string &str);

int get_robot_closest_to_their_goal(std::vector<int> robots, const roboteam_msgs::World &world = LastWorld::get());
int get_robot_closest_to_ball(std::vector<int> robots, const roboteam_msgs::World &world = LastWorld::get());

// ros::Publisher& get_rolecommand_publisher();
// ros::Publisher& get_roledirective_publisher();

template<
    class M
>
class GlobalPublisher {
public:
    GlobalPublisher(std::string topic, int queue = 100)
        : actualPub(n.advertise<M>(topic, queue)) {
        if (!GlobalPublisher<M>::pubPtr) {
            GlobalPublisher<M>::pubPtr = &actualPub;
        }
    }

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

} // rtt
