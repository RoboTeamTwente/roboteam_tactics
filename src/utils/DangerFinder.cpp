#include <cmath>
#include <map>

#include <boost/optional.hpp>

#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/DangerFinder.h"
#include "roboteam_tactics/generated/debug.h"

#define reason(msg, ...) if (reasoning != nullptr) { \
char buf[100]; \
snprintf(buf, 100, msg, ##__VA_ARGS__); \
*reasoning += buf; \
}

#define ORIENTATION_DENOMINATOR .99346 // magic
#define DISTANCE_DENOMINATOR 2.84605 // magic
#define HAS_BALL_DANGER 5.0
#define CAN_SEE_GOAL_DANGER 5.0
#define POTENTIAL_CROSS_DANGER 3.0

namespace rtt {
 
inline Vector get_goal() {
    return Vector(we_are_left() ? -3 : 3, 0);
}
   
inline Position get_opp(const Robot& bot) {
    return Position(bot.pos.x, bot.pos.y, bot.angle);
}   
   
bool we_are_left() {
    std::string our_field_side = "left";
    ros::param::get("our_field_side", our_field_side);

    return our_field_side == "left";
}

const DangerFactor can_see_our_goal = [](const Robot& bot, std::string* reasoning=nullptr) {
    std::vector<Vector> goal_points = we_are_left() ? GOAL_POINTS_LEFT : GOAL_POINTS_RIGHT;
    for (const Vector& goal : goal_points) {
        if (getObstacles(bot, goal, nullptr, true).empty()) {
            reason("[bot can see goal; adding 5.0]");
            return CAN_SEE_GOAL_DANGER;
        }
    }
    reason("[bot cannot see goal]");
    return 0.0;
};

const DangerFactor has_ball = [](const Robot& bot, std::string* reasoning=nullptr) {
    auto opt = getBallHolder();
    if ((bool) opt && !opt->second && bot.id == opt->first.id) {
        reason("[bot has ball; adding 5.0]");
        return HAS_BALL_DANGER;
    }
    reason("[bot does not have ball]");
    return 0.0;
};

const DangerFactor distance = [](const Robot& bot, std::string* reasoning=nullptr) {
    Vector goal = get_goal();
    Position opp = get_opp(bot);
    double x = (opp.location().dist(goal) - 9) / DISTANCE_DENOMINATOR;
    reason("[distance=%f; score=%f]", opp.location().dist(goal), x*x);
    return x*x;
    // score = ((dist - 9) / DISTANCE_DENOMINATOR)^2, 0 at dist=9, 10 at dist=0, quadratic growth
};

const DangerFactor orientation = [](const Robot& bot, std::string* reasoning=nullptr) {
    Vector goal = get_goal();
    Position opp = get_opp(bot);
    goal = goal - opp.location();
    double tgt_angle = goal.angle();
    double angle_diff = fabs(tgt_angle - bot.angle);
    double x = (angle_diff - M_PI) / ORIENTATION_DENOMINATOR;
    reason("[orientation diff=%f; score=%f]", angle_diff, (x*x)/3.0);
    return (x*x) / 3.0;
};

const DangerFactor potential_cross_recipient = [](const Robot& bot, std::string* reasoning=nullptr) {
    // TODO: incorporate chipped passes
    //Vector goal = get_goal();
    Position pos(bot.pos.x, bot.pos.y, bot.angle);
    auto holder = getBallHolder();
    if (!holder || holder->second) {
        // Opponent does not have ball
        return 0.0;
    }
    const auto other = holder->first;
    if (Vector(other.pos.x, other.pos.y) == pos.location()) {
        // Can't cross to yourself
        return 0.0;
    }
    
    if (!getObstacles(bot, Vector(other.pos.x, other.pos.y), nullptr, true).empty()) {
        // Other can't see the current bot
        return 0.0;
    }
    
    double my_score = danger_score(bot, DEFAULT_FACTORS, false);
    double other_score = danger_score(other, DEFAULT_FACTORS, false);
    if (my_score < other_score) {
        // Other is in better position
        return 0.0;
    }
    
    return POTENTIAL_CROSS_DANGER;
};

const std::vector<DangerFactor> DEFAULT_FACTORS({distance, orientation, can_see_our_goal, has_ball});

double danger_score(const Robot& bot, const std::vector<DangerFactor>& factors, 
                    bool include_cross, unsigned int preferred) {
    double score = 0.0;
    std::string reasoning;
    for (const DangerFactor& factor : factors) {
        score += factor(bot, &reasoning);
    }
    if (include_cross) {
        score += potential_cross_recipient(bot, &reasoning);
    }
    if (bot.id == preferred) {
        score += 10.0;
    }
    DEBUG_INFO_DANGER_FINDER("Reasoning for bot %d, final score=%f: %s", bot.id, score, reasoning.c_str());
    return score;
}

static std::vector<Robot> sorted_opponents(const roboteam_msgs::World& world, unsigned int preferred) {
    std::vector<Robot> bots = world.them;
    auto comp = [](const Robot& a, const Robot& b) {
        return danger_score(a) < danger_score(b);
    };
    std::vector<Robot> res;
    res.insert(res.cbegin(), bots.begin(), bots.end());
    std::sort<std::vector<Robot>::iterator, decltype(comp)>(res.begin(), res.end(), comp);
    return res;
}

boost::optional<Robot> most_dangerous_bot(unsigned int preferred) {
    roboteam_msgs::World world = LastWorld::get();
    if (world.them.size() < 1) return boost::optional<Robot>();
    auto it = sorted_opponents(world, preferred).end();
    it--;
    return *it;
}

boost::optional<Robot> second_most_dangerous_bot(unsigned int preferred) {
    roboteam_msgs::World world = LastWorld::get();
    if (world.them.size() < 2) return boost::optional<Robot>();
    auto it = sorted_opponents(world, preferred).end();
    it--;
    it--;
    return *it;
}
 
boost::optional<Robot> charging_bot() {
    return boost::optional<Robot>();
}
    
}
