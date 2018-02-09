#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/Parts.h"

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
 *       Default: 1.0
 *   - duration:
 *       Type: Double
 *       Descr: After how many seconds the robot should stop
 *       Default: 0.2
 *   - waitDribblerOff:
 *       Type: Double
 *       Descr: First wait until the dribbler has stopped for this amount of seconds
 *       Default: 0.0
 */
class Push : public Skill {
public:
	Push(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

	Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() { return "Push"; }
private:
    uint ROBOT_ID;
	int counter;
	int dur;
	double vel;
	int wait;
};


} // rtt
