#pragma once

#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/Section.h"
#include "roboteam_msgs/World.h"
#include <vector>
#include "boost/optional.hpp"
#include "roboteam_tactics/utils/GoalPartition.h"
#include "GetBall.h"

namespace rtt {

/**
 * \class ShootAtGoalV2
 * \brief See YAML
 */
/*
 * Descr: >
 *     Improved ShootAtGoal which finds the ideal place to aim, then shoots the ball there.
 *     This skill will calculate the largest area of the goal which the active robot can
 *     see unobstructed, and aim for the middle of that area.
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The robot which is to shoot
 */
class ShootAtGoalV2 : public Skill {
public:
    ShootAtGoalV2(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    
private:
    static constexpr double ACCEPTABLE_DEVIATION = .02; // rad
    static constexpr double KICK_VEL = 7.5; // .5 m/s safety margin
    GoalPartition partition;
    std::unique_ptr<GetBall> aimer;
};

}
