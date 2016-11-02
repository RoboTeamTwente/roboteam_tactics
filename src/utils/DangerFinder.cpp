#include "roboteam_tactics/utils/DangerFinder.h"
#include <cmath>
#include <map>
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_tactics/utils/utils.h"
#include <boost/optional.hpp>

namespace rtt {
 
inline Vector get_goal() {
    return Vector(we_are_left() ? -3 : 3, 0);
}
   
inline Position get_opp(const roboteam_msgs::WorldRobot& bot) {
    return Position(bot.pos.x, bot.pos.y, bot.angle);
}   
   
bool we_are_left() {
    return false;
}

bool can_see_our_goal(const roboteam_msgs::WorldRobot& bot) {
    std::vector<Vector> goal_points = we_are_left() ? GOAL_POINTS_LEFT : GOAL_POINTS_RIGHT;

    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->SetInt("me", bot.id);
    
    for (const Vector& goal : goal_points) {
        bb->SetDouble("x_coor", goal.x);
        bb->SetDouble("y_coor", goal.y);
        CanSeePoint csp("", bb);
        if (csp.Update() == bt::Node::Status::Success) {
            return true;
        }
    }
    
    return false;
}

bool potential_cross_recepient(const roboteam_msgs::WorldRobot& bot) {
    // TODO: incorporate chipped passes
    Vector goal = get_goal();
    Position pos(bot.pos.x, bot.pos.y, bot.angle);
    auto holder = getBallHolder();
    if (!holder || holder->second) {
        // Opponent does not have ball
        return false;
    }
    const auto other = holder->first;
    if (Vector(other.pos.x, other.pos.y) == pos.location()) {
        // Can't cross to yourself
        return false;
    }
    
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->SetInt("me", bot.id);
    bb->SetDouble("x_coor", other.pos.x);
    bb->SetDouble("y_coor", other.pos.y);
    CanSeePoint csp("", bb);
    if (csp.Update() != bt::Node::Status::Success) {
        // Other can't see the current bot
        return false;
    }
    
    double my_score = base_danger_score(bot);
    double other_score = base_danger_score(other);
    if (my_score < other_score) {
        // Other is in better position
        return false;
    }
    
    return true;
}

bool has_ball(const roboteam_msgs::WorldRobot& bot) {
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>();
    bb->SetInt("me", bot.id);
    bb->SetBool("our_team", false);
    IHaveBall ihb("", bb);
    return ihb.Update() == bt::Node::Status::Success;
}

#define DISTANCE_DENOMINATOR 2.84605 // magic
double distance_score(const roboteam_msgs::WorldRobot& bot) {
    Vector goal = get_goal();
    Position opp = get_opp(bot);
    double x = (opp.location().dist(goal) - 9) / DISTANCE_DENOMINATOR;
    return x*x;
    // score = ((dist - 9) / DISTANCE_DENOMINATOR)^2, 0 at dist=9, 10 at dist=0, quadratic growth
}

#define ORIENTATION_DENOMINATOR .99346 // magic
double orientation_score(const roboteam_msgs::WorldRobot& bot) {
    Vector goal = get_goal();
    Position opp = get_opp(bot);
    goal = goal - opp.location();
    double tgt_angle = goal.angle();
    double angle_diff = fabs(tgt_angle - bot.angle);
    double x = (angle_diff - M_PI) / ORIENTATION_DENOMINATOR;
    return (x*x) / 3.0;
}   


#define HAS_BALL_DANGER 5.0
#define CAN_SEE_GOAL_DANGER 5.0
double base_danger_score(const roboteam_msgs::WorldRobot& bot) {
    double score = 0.0;
    
    score += distance_score(bot);
    score += orientation_score(bot);
    if (can_see_our_goal(bot)) score += CAN_SEE_GOAL_DANGER;
    if (has_ball(bot)) score += HAS_BALL_DANGER;
    
    return score;
} 

double danger_score(const roboteam_msgs::WorldRobot& bot) {
    double base = base_danger_score(bot);
    double score = potential_cross_recepient(bot) ? base + 5 : base;
    return score;
}

void dump_scores(const roboteam_msgs::World& world) {
    for (const auto& bot : world.them) {
        double dist = distance_score(bot);
        double orient = orientation_score(bot);
        bool csg = can_see_our_goal(bot);
        bool ball = has_ball(bot);
        ROS_INFO("Bot %d -- dist=%f, orient=%f, csg=%d, ball=%d", bot.id, dist, orient, csg, ball);
    }
}

static std::vector<roboteam_msgs::WorldRobot> sorted_opponents(const roboteam_msgs::World& world) {
    std::vector<roboteam_msgs::WorldRobot> bots = world.them;
    auto comp = [](const roboteam_msgs::WorldRobot& a, const roboteam_msgs::WorldRobot& b) {
        return danger_score(a) < danger_score(b);
    };
    std::vector<roboteam_msgs::WorldRobot> res;
    res.insert(res.cbegin(), bots.begin(), bots.end());
    std::sort<std::vector<roboteam_msgs::WorldRobot>::iterator, decltype(comp)>(res.begin(), res.end(), comp);
    return res;
}

roboteam_msgs::WorldRobot most_dangerous_bot() {
    auto it = sorted_opponents(LastWorld::get()).end();
    it--;
    return *it;
}
 
boost::optional<roboteam_msgs::WorldRobot> charging_bot() {
    return boost::optional<roboteam_msgs::WorldRobot>();
}
    
}