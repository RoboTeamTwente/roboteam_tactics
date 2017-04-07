#include <cmath>
#include <map>

#include <boost/optional.hpp>

#include "roboteam_tactics/skills/ShootAtGoalV2.h"
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
 
RemoteDangerFinder dangerFinder;    
DangerFinder dangerFinderImpl;

inline Vector2 getGoal() {
    return Vector2(weAreLeft() ? -3 : 3, 0);
}
   
inline Position getOpp(const Robot& bot) {
    return Position(bot.pos.x, bot.pos.y, bot.angle);
}   

const DangerFactor canSeeOurGoal = [](const Robot& bot, std::string* reasoning=nullptr) {
    GoalPartition partition(weAreLeft());
    partition.calculatePartition(LastWorld::get(), Vector2(bot.pos));
    return (bool) partition.largestOpenSection();
};

const DangerFactor hasBall = [](const Robot& bot, std::string* reasoning=nullptr) {
    auto opt = getBallHolder();
    if ((bool) opt && !opt->second && bot.id == opt->first.id) {
        reason("[bot has ball; adding 5.0]");
        return HAS_BALL_DANGER;
    }
    reason("[bot does not have ball]");
    return 0.0;
};

const DangerFactor distance = [](const Robot& bot, std::string* reasoning=nullptr) {
    Vector2 goal = getGoal();
    Position opp = getOpp(bot);
    double x = (opp.location().dist(goal) - 9) / DISTANCE_DENOMINATOR;
    // reason("[distance=%f; score=%f]", opp.location().dist(goal), x*x);
    return x*x;
    // score = ((dist - 9) / DISTANCE_DENOMINATOR)^2, 0 at dist=9, 10 at dist=0, quadratic growth
};

const DangerFactor orientation = [](const Robot& bot, std::string* reasoning=nullptr) {
    Vector2 goal = getGoal();
    Position opp = getOpp(bot);
    goal = goal - opp.location();
    double tgt_angle = goal.angle();
    double angle_diff = fabs(tgt_angle - bot.angle);
    double x = (angle_diff - M_PI) / ORIENTATION_DENOMINATOR;
    // reason("[orientation diff=%f; score=%f]", angle_diff, (x*x)/3.0);
    return (x*x) / 3.0;
};

const DangerFactor potentialCrossRecipient = [](const Robot& bot, std::string* reasoning=nullptr) {
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
    
    double myScore = df_impl::dangerScore(bot, DEFAULT_FACTORS, false);
    double otherScore = df_impl::dangerScore(other, DEFAULT_FACTORS, false);
    if (myScore < otherScore) {
        // Other is in better position
        return 0.0;
    }
    
    return POTENTIAL_CROSS_DANGER;
};

const std::vector<DangerFactor> DEFAULT_FACTORS({distance/*, orientation, canSeeOurGoal, hasBall*/});

DangerFinder::DangerFinder() {
    running = false;
    _stop = false;
}

void DangerFinder::run(unsigned int delay) {
    ROS_INFO("DF start");
    _stop = false;
    runner = std::thread(&DangerFinder::runImpl, this, delay);
    runner.detach();
    running = true;
}

void DangerFinder::stop() {
    ROS_INFO("DF stop");
    _stop = true;
    runner.join();
    running = false;
}

bool DangerFinder::isRunning() const { return running; }

DangerResult DangerFinder::update() const {
    DangerResult newRes;
    newRes.dangerList = df_impl::sortedOpponents(LastWorld::get(), -1);
    int count = newRes.dangerList.size();
    newRes.mostDangerous = count > 0 ? boost::optional<Robot>(newRes.dangerList.at(count - 1)) : boost::optional<Robot>();
    newRes.secondMostDangerous = count > 1 ? boost::optional<Robot>(newRes.dangerList.at(count - 2)) : boost::optional<Robot>();
    newRes.charging = df_impl::chargingBot();
    return newRes;
}

void DangerFinder::runImpl(unsigned int delay) {
    ros::Rate rate(delay);
    while (!_stop) {
        DangerResult newRes = update();
                    
        while (!lock.try_lock()) {
           if (_stop) return; 
        }
        
        try {
            result = newRes;
        } catch (...) {
            lock.unlock();
            continue;
        }
        lock.unlock();
        rate.sleep();
    }
}

DangerResult DangerFinder::currentResult() {
    DangerResult res;
    lock.lock();
    try {
        res = result;
    } catch (...) {
        lock.unlock();
        throw std::runtime_error("DangerFinder::currentResult failed");
    }
    lock.unlock();
    return result;
}

DangerResult DangerFinder::getImmediateUpdate() const {
    return update();
}

namespace df_impl {
    
double dangerScore(const Robot& bot, const std::vector<DangerFactor>& factors, 
                    bool includeCross, unsigned int preferred) {
    double score = 0.0;
    std::string reasoning;
    for (const DangerFactor& factor : factors) {
        score += factor(bot, &reasoning);
    }
    if (includeCross) {
        score += potentialCrossRecipient(bot, &reasoning);
    }
    if (bot.id == preferred) {
        score += 10.0;
    }
    // DEBUG_INFO_DANGER_FINDER("Reasoning for bot %d, final score=%f: %s", bot.id, score, reasoning.c_str());
    return score;
}

std::vector<Robot> sortedOpponents(const roboteam_msgs::World& world, unsigned int preferred) {
    std::vector<Robot> bots = world.them;
    auto comp = [](const Robot& a, const Robot& b) {
        return dangerScore(a) < dangerScore(b);
    };
    std::vector<Robot> res;
    res.insert(res.cbegin(), bots.begin(), bots.end());
    std::sort<std::vector<Robot>::iterator, decltype(comp)>(res.begin(), res.end(), comp);
    return res;
}

boost::optional<Robot> mostDangerousBot(unsigned int preferred) {
    roboteam_msgs::World world = LastWorld::get();
    if (world.them.size() < 1) return boost::optional<Robot>();
    auto it = sortedOpponents(world, preferred).end();
    it--;
    return *it;
}

boost::optional<Robot> secondMostDangerousBot(unsigned int preferred) {
    roboteam_msgs::World world = LastWorld::get();
    if (world.them.size() < 2) return boost::optional<Robot>();
    auto it = sortedOpponents(world, preferred).end();
    it--;
    it--;
    return *it;
}
 
boost::optional<Robot> chargingBot() {
    return boost::optional<Robot>();
}

bool weAreLeft() {
    std::string tgt;
    get_PARAM_OUR_SIDE(tgt, false);
    return tgt == "left";
}

}

static inline void ensureRunning() {
    if (!dangerFinder.isRunning())
        dangerFinder.run(100);
}

bool weAreLeft() { ensureRunning(); return df_impl::weAreLeft(); }
boost::optional<Robot> chargingBot() { ensureRunning(); return dangerFinder.currentResult().charging; }
boost::optional<Robot> mostDangerousBot() { ensureRunning(); return dangerFinder.currentResult().mostDangerous; }
boost::optional<Robot> secondMostDangerousBot() { ensureRunning(); return dangerFinder.currentResult().secondMostDangerous; }
 
void RemoteDangerFinder::run(unsigned int delay = 100) {}

void RemoteDangerFinder::stop() {}

bool RemoteDangerFinder::isRunning() const { return true; }

inline DangerResult convert(const DFService::Response& res) {
    return {
        res.charging.present ? boost::optional<Robot>(res.charging.robot) : boost::none,
        res.mostDangerous.present ? boost::optional<Robot>(res.mostDangerous.robot) : boost::none,
        res.secondMostDangerous.present ? boost::optional<Robot>(res.secondMostDangerous.robot) : boost::none,
        res.robots
    };
}

inline DangerResult fetch(bool immediate, bool mostDangerousOnly) {
    if (ros::this_node::getName() == "/StrategyNode") {
        return immediate ? dangerFinderImpl.getImmediateUpdate() : dangerFinderImpl.currentResult();
    }
    DFService::Request req;
    DFService::Response res;
    
    req.immediate = immediate;
    req.mostDangerousOnly = mostDangerousOnly;
    
    if (!ros::service::call("dangerFinder", req, res)) {
        return DangerResult();
    }
    
    return convert(res);
}

DangerResult RemoteDangerFinder::currentResult() {
    return fetch(false, false);
}

DangerResult RemoteDangerFinder::getImmediateUpdate() const {
    return fetch(true, false);
}
   
}
