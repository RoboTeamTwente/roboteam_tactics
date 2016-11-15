#pragma once

#include <boost/optional.hpp>
#include <vector>
#include <algorithm>

#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"

namespace rtt {
    
using Vector = roboteam_utils::Vector2;    
using Position = roboteam_utils::Position;    
using Robot = roboteam_msgs::WorldRobot;

/*
class DangerFactor {
public:
    virtual ~DangerFactor() {}
    virtual double calc_score(const Robot& bot, std::string* reasoning = nullptr) = 0;
};
*/

typedef std::function<double(const Robot&, std::string*)> DangerFactor;
extern const DangerFactor can_see_our_goal;
extern const DangerFactor potential_cross_recipient;
extern const DangerFactor has_ball;
extern const DangerFactor distance;
extern const DangerFactor orientation;

extern const std::vector<DangerFactor> DEFAULT_FACTORS;

const std::vector<Vector> GOAL_POINTS_LEFT({
    Vector(-3, .35),
    Vector(-3, 0),
    Vector(-3, -.35)
});
  
const std::vector<Vector> GOAL_POINTS_RIGHT({
    Vector(3, .35),
    Vector(3, 0),
    Vector(3, -.35)
});


std::vector<Vector> our_goal();

bool we_are_left();

double danger_score(const Robot& bot, const std::vector<DangerFactor>& factors = DEFAULT_FACTORS, 
                    bool include_cross = true, unsigned int preferred = 999);
    
void dump_scores(const roboteam_msgs::World& world);    
    
boost::optional<Robot> charging_bot();
boost::optional<Robot> most_dangerous_bot(unsigned int preferred = 999);
boost::optional<Robot> second_most_dangerous_bot(unsigned int preferred = 999);
 

   
}
