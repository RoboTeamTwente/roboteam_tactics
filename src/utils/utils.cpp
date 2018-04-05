#include <boost/range/join.hpp>
#include <ros/ros.h>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"

#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/world_analysis.h"

namespace rtt {

/**
 * Gets all the nodes subscribed to topic topic, relative
 * to the current node.
 */
std::vector<std::string> getNodesSubscribedTo(std::string topic) {
    // Adapted from ros::master::getNodes()
    // Do the xmlrpc request to master
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getSystemState", args, result, payload, true)) {
        // Return an empty list if something went wrong
        return {};
    }

    // See: http://wiki.ros.org/ROS/Master_API
    // and then getSystemState for the format
    std::vector<std::string> nodes;
    XmlRpc::XmlRpcValue subscribers = payload[1];
    for (int i = 0; i < subscribers.size(); ++i) {
        XmlRpc::XmlRpcValue topicInfo = subscribers[i];
        XmlRpc::XmlRpcValue topicName = topicInfo[0];
        XmlRpc::XmlRpcValue topicSubscribers = topicInfo[1];

        if (topicName == topic) {
            for (int j = 0; j < topicSubscribers.size(); ++j) {
                nodes.push_back(topicSubscribers[j]);
            }
            break;
        }
    }

    return nodes;
}

std::string getMyNamespace() {
    std::string ns = ros::this_node::getNamespace();

    if (ns.size() >= 2) {
        if (ns.substr(0, 2) == "//") {
            ns = ns.substr(1, ns.size() - 1);
        }
    }

    if (ns.size() >= 1) {
        if (ns.at(ns.size() - 1) != '/') {
            ns += "/";
        }
    }

    return ns;
}

double GetTargetAngle(Vector2 startPos, std::string target, int theirID, bool target_our_team) {
    roboteam_msgs::World w = LastWorld::get();
    // boost::optional<roboteam_msgs::WorldRobot> bot = lookup_bot(myID, our_team, &w);
    // roboteam_msgs::WorldRobot robot;
    // if (bot) {robot = *bot;}
    // else {
    //     ROS_WARN("utils/GetTargetAngle: cannot find robot");
    //     return 0.0;
    // }

    if (target == "fieldcenter") {
        Vector2 posDiff = Vector2(0.0, 0.0) - startPos;
        return posDiff.angle();
    }

    if (target == "theirgoal") {
        Vector2 theirGoalPos = LastWorld::get_their_goal_center();
        double targetAngle = (theirGoalPos - startPos).angle();
        return targetAngle;
    }
    if (target == "ourgoal") {
        Vector2 ourGoalPos = LastWorld::get_our_goal_center();
        double targetAngle = (ourGoalPos - startPos).angle();
        return targetAngle;
    }
    if (target == "robot") {
        boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(theirID, target_our_team, w);
        Vector2 theirPos;
        if (findBot) {
            theirPos = Vector2(findBot->pos);
        } else {
            ROS_WARN("GetTargetAngle (utils.cpp) could not find robot");
            return 0.0;
        }
        double targetAngle = (theirPos - startPos).angle();
        return targetAngle;
    }
    if (target == "param") {
    	int tgt;
    	if (ros::param::get("AimTargetBot", tgt)) {

            boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(tgt);
            Vector2 theirPos;
            if (findBot) {
                theirPos = Vector2(findBot->pos);
            } else {
                ROS_WARN("GetTargetAngle (utils.cpp) could not find robot");
                return 0.0;
            }

    		double targetAngle = (theirPos - startPos).angle();
    		return targetAngle;
    	}
    }
    ROS_WARN("cannot find TargetAngle, maybe your input arguments are wrong?");
    return 0.0;
}

Vector2 getTargetPos(std::string target, int theirID, bool target_our_team) {
    roboteam_msgs::World w = LastWorld::get();

    if (target == "fieldcenter") {
        return Vector2(0.0, 0.0);
    }

    if (target == "theirgoal") {
        return LastWorld::get_their_goal_center();
    }
    if (target == "ourgoal") {
        return LastWorld::get_our_goal_center();
    }
    if (target == "robot") {

        boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(theirID, target_our_team, w);
        Vector2 theirPos;
        if (findBot) {
            theirPos = Vector2(findBot->pos);
        } else {
            ROS_WARN("GetTargetPos (utils.cpp) could not find robot");
            return Vector2(0.0, 0.0);
        }

        return theirPos;
    }
    ROS_WARN("cannot find TargetAngle, maybe your input arguments are wrong?");
    return Vector2(0.0, 0.0);
}

boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>> getBallHolder() {
    const auto& ball = LastWorld::get().ball;
    for (auto& bot : LastWorld::get().us) {
        if (bot_has_ball(bot, ball)) {
            return boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>>(std::make_pair(bot, true));
        }
    }
    for (auto& bot : LastWorld::get().them) {
        if (bot_has_ball(bot, ball)) {
            return boost::optional<std::pair<roboteam_msgs::WorldRobot, bool>>(std::make_pair(bot, false));
        }
    }
    return boost::none;
}
std::vector<roboteam_msgs::WorldRobot> getObstacles(const roboteam_msgs::WorldRobot& bot,
                                                    const Vector2& point,
                                                    const roboteam_msgs::World* world_ptr,
                                                    bool sight_only,
													bool ignore_both_ends) {
    return getObstacles(Vector2(bot.pos.x, bot.pos.y), point, world_ptr, sight_only, ignore_both_ends);
}

std::vector<roboteam_msgs::WorldRobot> getObstacles(const Vector2& bot_pos,
                                                    const Vector2& point,
                                                    const roboteam_msgs::World* world_ptr,
                                                    bool sight_only,
													bool ignore_both_ends) {
	// This used to be implemented here, but was moved to roboteam_utils/src/world_analysis.cpp
    return getObstaclesBetweenPoints(bot_pos, point, world_ptr, sight_only, ignore_both_ends);
}

std::vector<roboteam_msgs::WorldRobot> getRobotsInCone(const roboteam_msgs::World& world, const Cone& cone) {
    std::vector<roboteam_msgs::WorldRobot> vec;
    for (const auto& bot : world.us) {
        if (cone.IsWithinCone(Vector2(bot.pos), .9)) {
            vec.push_back(bot);
        }
    }
    for (const auto& bot : world.them) {
        if (cone.IsWithinCone(Vector2(bot.pos), .9)) {
            vec.push_back(bot);
        }
    }
    return vec;
}

Vector2 predictBallPos(double seconds) {
    roboteam_msgs::World w = LastWorld::get();
    Vector2 ballPosNow(w.ball.pos);
    Vector2 ballVelNow(w.ball.vel);
    Vector2 predictedBallPos = ballPosNow + ballVelNow*seconds;
    return predictedBallPos;
}

Vector2 predictRobotPos(uint robot_id, bool our_team, double seconds) {
    roboteam_msgs::World w = LastWorld::get();
    Vector2 robotPosNow;
    Vector2 robotVelNow;
    if (our_team) {
        robotPosNow = w.us.at(robot_id).pos;
        robotVelNow = w.us.at(robot_id).vel;
    } else {
        robotPosNow = w.them.at(robot_id).pos;
        robotVelNow = w.them.at(robot_id).vel;
    }
    Vector2 predictedBallPos = robotPosNow + robotVelNow*seconds;
    return predictedBallPos;
}

bool bot_has_ball(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::WorldBall& ball) {

    Vector2 ball_vec(ball.pos), bot_vec(bot.pos);
    Vector2 ball_norm = (ball_vec - bot_vec);

    double dist = ball_norm.length();
    double angle = ball_norm.angle();
    
    // Within 11 cm and .3 radians (of center of dribbler)
    return dist <= .13 && fabs(angle - bot.angle) <= .4;
}

void print_blackboard(const bt::Blackboard::Ptr bb, std::ostream& out) {
    out << "Blackboard:\n";
    for (const auto& pair : bb->getBools()) {
        out << "\t" << pair.first << ": " << (pair.second ? "true" : "false") << "\n";
    }
    for (const auto& pair : bb->getInts()) {
        out << "\t" << pair.first << ": " << pair.second << "\n";
    }
    for (const auto& pair : bb->getDoubles()) {
        out << "\t" << pair.first << ": " << pair.second << "\n";
    }
    for (const auto& pair : bb->getFloats()) {
        out << "\t" << pair.first << ": " << pair.second << "\n";
    }
    for (const auto& pair : bb->getStrings()) {
        out << "\t" << pair.first << ": \"" << pair.second << "\"\n";
    }
}

void merge_blackboards(bt::Blackboard::Ptr target, const bt::Blackboard::Ptr extras) {
    for (const auto& pair : extras->getBools()) {
        target->SetBool(pair.first, pair.second);
    }
    for (const auto& pair : extras->getInts()) {
        target->SetInt(pair.first, pair.second);
    }
    for (const auto& pair : extras->getDoubles()) {
        target->SetDouble(pair.first, pair.second);
    }
    for (const auto& pair : extras->getFloats()) {
        target->SetFloat(pair.first, pair.second);
    }
    for (const auto& pair : extras->getStrings()) {
        target->SetString(pair.first, pair.second);
    }

}

namespace {
    thread_local std::random_device rd;
    thread_local std::mt19937 rng(rd());
}

int get_rand_int(int max) {
    return get_rand_int(0, max);
}

int get_rand_int(int min, int max) {
    std::uniform_int_distribution<>dis(min, max - 1)  ;
    return dis(rng);
}

float get_rand_real(float max) {
    return get_rand_real(0, max);
}

float get_rand_real(float min, float max) {
    std::uniform_real_distribution<>dis(min, max)  ;
    return dis(rng);
}

time_point now() {
    return std::chrono::steady_clock::now();
}

seconds time_difference_seconds(time_point start, time_point end) {
    return std::chrono::duration_cast<std::chrono::seconds>(end - start);
}

milliseconds time_difference_milliseconds(time_point start, time_point end) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
}

std::string describe_status(bt::Node::Status status) {
    switch (status) {
        case bt::Node::Status::Success: return "Success";
        case bt::Node::Status::Failure: return "Failure";
        case bt::Node::Status::Invalid: return "Invalid";
        case bt::Node::Status::Running: return "Running";
        default: return "<<Something is very, very broken>>";
    }
}

std::string get_our_side() {
    std::string tgt;
    get_PARAM_OUR_SIDE(tgt, false);
    return tgt;
}

bool is_digits(const std::string &str) {
    return std::all_of(str.begin(), str.end(), ::isdigit);
}

roboteam_msgs::RobotCommand stop_command(unsigned int id) {
    roboteam_msgs::RobotCommand cmd;
    cmd.id = id;
    cmd.active = true;
	cmd.x_vel = 0.0;
	cmd.y_vel = 0.0;
	cmd.w = 0.0;

	cmd.dribbler=false;
	cmd.kicker=false;
	cmd.kicker_vel=0.0;
	cmd.kicker_forced=false;
	cmd.chipper=false;
	cmd.chipper_vel=0.0;
	cmd.chipper_forced=false;
    return cmd;
}

boost::optional<int> get_robot_closest_to_point(std::vector<int> robots, const roboteam_msgs::World& world, const Vector2& point) {
    int closest_robot = -1;
    if (robots.size()==0){
        ROS_ERROR("you gave get_robot_closest_to_point an empty list too chose from you silly");
    }
    double closest_robot_ds = std::numeric_limits<double>::max();

    for (roboteam_msgs::WorldRobot worldRobot : world.us) {
        Vector2 pos(worldRobot.pos);
        if (std::find(robots.begin(), robots.end(), worldRobot.id) != robots.end()) {
            if ((pos - point).length() < closest_robot_ds) {
                closest_robot = worldRobot.id;
                closest_robot_ds = (pos - point).length();
            }
        }
    }
    return closest_robot == -1 ? boost::none : boost::optional<int>(closest_robot);
}

boost::optional<int> get_robot_closest_to_point(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point) {
    int closest_robot = -1;
    double closest_robot_ds = std::numeric_limits<double>::max();

    for (roboteam_msgs::WorldRobot worldRobot : robots) {
        Vector2 pos(worldRobot.pos);

        if ((pos - point).length() < closest_robot_ds) {
           closest_robot = worldRobot.id;
           closest_robot_ds = (pos - point).length();
        }
    }

    return closest_robot == -1 ? boost::none : boost::optional<int>(closest_robot);
}

boost::optional<int> predict_robot_closest_to_point(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point, double t_ahead) {
    int closest_robot = -1;
    double closest_robot_ds = std::numeric_limits<double>::max();

    for (roboteam_msgs::WorldRobot worldRobot : robots) {
        Vector2 pos(worldRobot.pos);
        Vector2 vel(worldRobot.vel);
        pos = pos + vel.scale(t_ahead);

        if ((pos - point).length() < closest_robot_ds) {
           closest_robot = worldRobot.id;
           closest_robot_ds = (pos - point).length();
        }
    }

    return closest_robot == -1 ? boost::none : boost::optional<int>(closest_robot);
}


boost::optional<int> get_robot_closest_to_ball(std::vector<int> robots) {
    roboteam_msgs::World lw = LastWorld::get();
    return get_robot_closest_to_ball(robots, lw);
}

boost::optional<int> get_robot_closest_to_ball(std::vector<int> robots, const roboteam_msgs::World &world) {
    Vector2 ball_pos(world.ball.pos);
    return get_robot_closest_to_point(robots, world, ball_pos);
}

boost::optional<int> get_robot_closest_to_their_goal(std::vector<int> robots) {
    roboteam_msgs::World lw = LastWorld::get();
    return get_robot_closest_to_their_goal(robots, lw);
}

// int get_robot_closest_to_our_goal(std::vector<roboteam_msgs::WorldRobot> robots) {
//     Vector2 ourGoalCenter = LastWorld::get_our_goal_center();
//     return get_robot_closest_to_point(robots, ourGoalCenter);
// }

boost::optional<int> get_robot_closest_to_our_goal(std::vector<int> robots) {
    roboteam_msgs::World lw = LastWorld::get();
    Vector2 ourGoal = LastWorld::get_our_goal_center();
    return get_robot_closest_to_point(robots, lw, ourGoal);
    // return get_robot_closest_to_their_goal(robots, lw);
}


boost::optional<int> get_robot_closest_to_their_goal(std::vector<int> robots, const roboteam_msgs::World &world) {
    int side = LastWorld::get_our_goal_center().x;

    int furthest_robot = -1;
    double furthest_robot_side_dist = -std::numeric_limits<double>::max();
    for (roboteam_msgs::WorldRobot worldRobot : world.us) {
        double this_robot_side_dist = std::abs(worldRobot.pos.x - side);
        if (this_robot_side_dist > furthest_robot_side_dist) {
            if (std::find(robots.begin(), robots.end(), worldRobot.id) != robots.end()) {
                furthest_robot = worldRobot.id;
                furthest_robot_side_dist = this_robot_side_dist;
            }
        }
    }

    return furthest_robot == -1 ? boost::none : boost::optional<int>(furthest_robot);
}










int getClosestOppToPoint(Vector2 testPosition, roboteam_msgs::World world) {

        if (world.them.size() == 0) {
            return 0.0;
        }

        double shortestDistance = (Vector2(world.them.at(0).pos) - testPosition).length();
        int closestOppToPoint = 0;
        for (size_t i = 1; i < world.them.size(); i++) {
            double testDistance = (Vector2(world.them.at(i).pos) - testPosition).length();
            if (testDistance < shortestDistance) {
                shortestDistance = testDistance;
                closestOppToPoint = i;
            }
        }
        return closestOppToPoint;
    }







std::vector<roboteam_msgs::WorldRobot> getAllBots(const roboteam_msgs::World& world) {
    std::vector<roboteam_msgs::WorldRobot> bots;
    bots.insert(bots.end(), world.us.begin(), world.us.end());
    bots.insert(bots.end(), world.them.begin(), world.them.end());
    return bots;
}

std::vector<TeamRobot> getAllTeamBots(const roboteam_msgs::World& world) {
    std::vector<TeamRobot> bots;
    for (const auto& bot : world.us) {
        bots.push_back({bot.id, true});
    }
    for (const auto& bot : world.them) {
        bots.push_back({bot.id, false});
    }
    return bots;
}

boost::optional<roboteam_msgs::WorldRobot> getWorldBot(unsigned int id, bool ourTeam, const roboteam_msgs::World& world) {
    std::vector<roboteam_msgs::WorldRobot> bots = ourTeam ? world.us : world.them;
    for (const auto& bot : bots) {
        if (bot.id == id) {
            return boost::optional<roboteam_msgs::WorldRobot>(bot);
        }
    }
    return boost::none;
}

boost::optional<TeamRobot> getTeamBot(unsigned int id, bool ourTeam, const roboteam_msgs::World& world) {
    std::vector<roboteam_msgs::WorldRobot> bots = ourTeam ? world.us : world.them;
    for (const auto& bot : bots) {
        if (bot.id == id) {
            return boost::optional<TeamRobot>({bot.id, ourTeam});
        }
    }
    return boost::none;
}

boost::optional<roboteam_msgs::WorldRobot> getBotFromDangerList(unsigned dangerIndex) {
	const auto& world = LastWorld::get();
	if (dangerIndex >= world.dangerList.size()) {
		return boost::none;
	}
	return world.dangerList.at(dangerIndex);
}

bool weAreLeft() {
    std::string tgt;
    get_PARAM_OUR_SIDE(tgt, false);
    return tgt == "left";
}

std::vector<int> getAvailableRobots(roboteam_msgs::World const & world) {
    std::vector<int> dealerRobots = RobotDealer::getClaimedRobots();
    std::vector<int> availableRobots;

    for (auto const & bot : world.us) {
        if (bot.id != static_cast<unsigned int>(RobotDealer::get_keeper())
                && std::find(dealerRobots.begin(), dealerRobots.end(), bot.id) == dealerRobots.end()) {
            availableRobots.push_back(bot.id);
        }
    }

    return availableRobots;
}

bool IsWithinField(Vector2 point) {
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    double fieldLimitX = field.field_length / 2.0;
    double fieldLimitY = field.field_width / 2.0;
    if (point.x > fieldLimitX || point.x < -fieldLimitX || point.y > fieldLimitY || point.y < -fieldLimitY) {
        return false;
    } else {
        return true;
    }
}


// Calculates the distance between the testPosition and a robot
    double calcDistGoalToRobot(int robot, roboteam_msgs::World world) {
        Vector2 goalPos = LastWorld::get_their_goal_center();
        Vector2 robotPos = Vector2(world.us.at(robot).pos);

        double distGoalToRobot = (robotPos - goalPos).length();

        return distGoalToRobot;
    }

} // rtt
