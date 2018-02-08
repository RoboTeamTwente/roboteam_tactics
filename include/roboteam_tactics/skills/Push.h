#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

/**
 * \class Push
 * \brief See YAML
 */
/*
 * Descr: Push the ball
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *   - velocity:
 *       Type: Double
 *       Descr: The velocity to push the ball with
 *       Default: 0.5
 *   - duration:
 *       Type: Double
 *       Descr: After how many seconds the robot should stop
 *       Default: 0.5
 */
class Push : public Skill {
public:
	Push(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	
	boost::optional<roboteam_msgs::RobotCommand> getVelCommand();
	
	Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() { return "Push"; }
private:
    
    uint ROBOT_ID;
};


} // rtt
