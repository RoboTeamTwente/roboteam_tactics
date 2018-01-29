#pragma once

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "GoToPos.h"

namespace rtt {

/**
 * \class Approach
 * \brief See YAML
 */
/*
 * Descr: TODO
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *
 *   - xGoal:
 *       Type: Double
 *       Descr: The x coordinate of the location the robot should approach
 *
 *   - yGoal:
 *       Type: Double
 *       Descr: The y coordinate of the location the robot should approach
 *
 *   - approachDistance:
 *       Type: Double
 *       Descr: The distance the robot should keep from the goal location in meters
 *       Default: .3
 */
 
class Approach : public Skill {
    public:
    Approach(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
    
    private:
    static constexpr double DEFAULT_DISTANCE = .3;
    std::unique_ptr<GoToPos> gtp;
};
    
}
