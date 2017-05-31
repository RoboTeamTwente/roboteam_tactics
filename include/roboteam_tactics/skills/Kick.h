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
 *   - forced:
 *       Type: Bool
 *       Descr: If true, sets the 'forced' flag in the robot commands
 *       Default: false
 *   - kickVel:
 *       Type: Double
 *       Descr: The velocity to kick the ball with
 *       Default: 4.0
 *   - pushFirst:
 *       Type: Bool
 *       Descr: When true, first push the ball forward a little bit to ensure it is in contact with the kicker
 *       Default: false
 */
class Kick : public Skill {
public:
	Kick(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize() override;
	Status Update() override;
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
    bool goneForward;

    void doKick();
    void goForward();
};


} // rtt
