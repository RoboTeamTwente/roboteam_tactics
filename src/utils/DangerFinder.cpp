#include <cmath>
#include <map>

#include <boost/optional.hpp>

#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_tactics/conditions/IHaveBall.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/DangerFinder.h"
#include "roboteam_utils/constants.h"

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
 
DangerFinder danger_finder;    
    
inline Vector2 get_goal() {
    return Vector2(we_are_left() ? -3 : 3, 0);
}
   
inline Position get_opp(const Robot& bot) {
    return Position(bot.pos.x, bot.pos.y, bot.angle);
}   

const DangerFactor can_see_our_goal = [](const Robot& bot, std::string* reasoning=nullptr) {
    std::vector<Vector2> goal_points = we_are_left() ? GOAL_POINTS_LEFT : GOAL_POINTS_RIGHT;
    for (const Vector2& goal : goal_points) {
        if (getObstacles(bot, goal, nullptr, true).empty()) {
            // reason("[bot can see goal; adding 5.0]");
            return CAN_SEE_GOAL_DANGER;
        }
    }
    // reason("[bot cannot see goal]");
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
    Vector2 goal = get_goal();
    Position opp = get_opp(bot);
    double x = (opp.location().dist(goal) - 9) / DISTANCE_DENOMINATOR;
    // reason("[distance=%f; score=%f]", opp.location().dist(goal), x*x);
    return x*x;
    // score = ((dist - 9) / DISTANCE_DENOMINATOR)^2, 0 at dist=9, 10 at dist=0, quadratic growth
};

const DangerFactor orientation = [](const Robot& bot, std::string* reasoning=nullptr) {
    Vector2 goal = get_goal();
    Position opp = get_opp(bot);
    goal = goal - opp.location();
    double tgt_angle = goal.angle();
    double angle_diff = fabs(tgt_angle - bot.angle);
    double x = (angle_diff - M_PI) / ORIENTATION_DENOMINATOR;
    // reason("[orientation diff=%f; score=%f]", angle_diff, (x*x)/3.0);
    return (x*x) / 3.0;
};

const DangerFactor potential_cross_recipient = [](const Robot& bot, std::string* reasoning=nullptr) {
    // TODO: incorporate chipped passes
    //Vector2 goal = get_goal();
    Position pos(bot.pos.x, bot.pos.y, bot.angle);
    auto holder = getBallHolder();
    if (!holder || holder->second) {
        // Opponent does not have ball
        return 0.0;
    }
    const auto other = holder->first;
    if (Vector2(other.pos.x, other.pos.y) == pos.location()) {
        // Can't cross to yourself
        return 0.0;
    }
    
    if (!getObstacles(bot, Vector2(other.pos.x, other.pos.y), nullptr, true).empty()) {
        // Other can't see the current bot
        return 0.0;
    }
    
    double my_score = df_impl::danger_score(bot, DEFAULT_FACTORS, false);
    double other_score = df_impl::danger_score(other, DEFAULT_FACTORS, false);
    if (my_score < other_score) {
        // Other is in better position
        return 0.0;
    }
    
    return POTENTIAL_CROSS_DANGER;
};

const std::vector<DangerFactor> DEFAULT_FACTORS({distance/*, orientation, can_see_our_goal, has_ball*/});

DangerFinder::DangerFinder() {
    running = false;
    _stop = false;
}

void DangerFinder::run(unsigned int delay) {
    ROS_INFO("DF start");
    _stop = false;
    runner = std::thread(&DangerFinder::_run, this, delay);
    runner.detach();
    running = true;
}

void DangerFinder::stop() {
    ROS_INFO("DF stop");
    _stop = true;
    runner.join();
    running = false;
}

bool DangerFinder::is_running() const { return running; }

DangerResult DangerFinder::update() const {
    DangerResult new_res;
    new_res.danger_list = df_impl::sorted_opponents(LastWorld::get(), -1);
    int count = new_res.danger_list.size();
    new_res.most_dangerous = count > 0 ? boost::optional<Robot>(new_res.danger_list.at(count - 1)) : boost::optional<Robot>();
    new_res.second_most_dangerous = count > 1 ? boost::optional<Robot>(new_res.danger_list.at(count - 2)) : boost::optional<Robot>();
    new_res.charging = df_impl::charging_bot();
    return new_res;
}

void DangerFinder::_run(unsigned int delay) {
    while (!_stop) {
        DangerResult new_res = update();
                    
        while (!lock.try_lock()) {
           if (_stop) return; 
        }
        
        try {
            result = new_res;
        } catch (...) {
            lock.unlock();
            continue;
        }
        lock.unlock();
    }
}

DangerResult DangerFinder::current_result() {
    DangerResult res;
    lock.lock();
    try {
        res = result;
    } catch (...) {
        lock.unlock();
        throw std::runtime_error("DangerFinder::current_result failed");
    }
    lock.unlock();
    return result;
}

DangerResult DangerFinder::get_immediate_update() const {
    return update();
}

namespace df_impl {
    
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
    // DEBUG_INFO_DANGER_FINDER("Reasoning for bot %d, final score=%f: %s", bot.id, score, reasoning.c_str());
    return score;
}

std::vector<Robot> sorted_opponents(const roboteam_msgs::World& world, unsigned int preferred) {
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

bool we_are_left() {
    //std::string tgt;
    //get_PARAM_OUR_SIDE(tgt, false);
    //return tgt == "left";
    return false;
}

std::vector<Vector2> our_goal() {
    return we_are_left() ? GOAL_POINTS_LEFT : GOAL_POINTS_RIGHT;
}

}

static inline void ensure_running() {
    if (!danger_finder.is_running())
        danger_finder.run();
}

std::vector<Vector2> our_goal() { ensure_running(); return df_impl::our_goal(); }
bool we_are_left() { ensure_running(); return df_impl::we_are_left(); }
boost::optional<Robot> charging_bot() { ensure_running(); return danger_finder.current_result().charging; }
boost::optional<Robot> most_dangerous_bot() { ensure_running(); return danger_finder.current_result().most_dangerous; }
boost::optional<Robot> second_most_dangerous_bot() { ensure_running(); return danger_finder.current_result().second_most_dangerous; }
    
}
