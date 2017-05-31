#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

/**
 * \class Kick
 * \brief See YAML
 */
/*
 * Descr: Kick the ball
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 *   - wait_for_signal:
 *       Type: Bool
 *       Descr: If true, the robot will only kick when the ready_to_pass global ros param is true
 *   - kickVel:
 *       Type: Double
 *       Descr: The velocity to kick the ball with
 *       Default: 4.0
 */
class Kick : public Skill {
public:
	Kick(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize() override;
	Status Update() override;
// yoooooooo
    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() override { return "Kick"; }
private:
	int robotID;

    Vector2 ballStartPos;
    Vector2 oldBallVel;
    int cycleCounter;
    time_point startTime;
};


} // rtt
