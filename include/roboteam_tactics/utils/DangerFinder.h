#pragma once

#include <boost/optional.hpp>
#include <vector>
#include <algorithm>
#include <mutex>
#include <thread>

#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

namespace rtt {
    
using Vector = roboteam_utils::Vector2;    
using Position = roboteam_utils::Position;    
using Robot = roboteam_msgs::WorldRobot;

typedef std::function<double(const Robot&, std::string*)> DangerFactor;
extern const DangerFactor can_see_our_goal;
extern const DangerFactor potential_cross_recipient;
extern const DangerFactor has_ball;
extern const DangerFactor distance;
extern const DangerFactor orientation;

extern const std::vector<DangerFactor> DEFAULT_FACTORS;

const std::vector<Vector> GOAL_POINTS_LEFT({
    Vector(-4.5, .35),
    Vector(-4.5, 0),
    Vector(-4.5, -.35)
});
  
const std::vector<Vector> GOAL_POINTS_RIGHT({
    Vector(4.5, .35),
    Vector(4.5, 0),
    Vector(4.5, -.35)
});

typedef struct {
    boost::optional<Robot> charging;
    boost::optional<Robot> most_dangerous;
    boost::optional<Robot> second_most_dangerous;
    std::vector<Robot> danger_list;
} DangerResult;

class DangerFinder {
    public:
    DangerFinder();
    
    void run(unsigned int delay = 100);
    void stop();
    bool is_running() const;
    
    DangerResult current_result();
    DangerResult get_immediate_update() const;
    
    private:
    void _run(unsigned int delay);
    DangerResult update() const;
    
    DangerResult result;
    bool running;
    volatile bool _stop;
    std::mutex lock;
    std::thread runner;
};

extern DangerFinder danger_finder;

namespace df_impl {
    std::vector<Vector> our_goal();

    bool we_are_left();

    double danger_score(const Robot& bot, const std::vector<DangerFactor>& factors = DEFAULT_FACTORS, 
                    bool include_cross = true, unsigned int preferred = 999);
    
    void dump_scores(const roboteam_msgs::World& world);    
    
    boost::optional<Robot> charging_bot();
    boost::optional<Robot> most_dangerous_bot(unsigned int preferred = 999);
    boost::optional<Robot> second_most_dangerous_bot(unsigned int preferred = 999);
    std::vector<Robot> sorted_opponents(const roboteam_msgs::World& world, unsigned int preferred);
}

std::vector<Vector> our_goal();
bool we_are_left();
boost::optional<Robot> charging_bot();
boost::optional<Robot> most_dangerous_bot();
boost::optional<Robot> second_most_dangerous_bot();
   
}
