#pragma once

#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

/**
 * \class GenevaKick
 * \brief See YAML
 */
/*
 * Descr: Kick the ball and uses the genevadrive
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
 *       Default: 10.0
 *   - pushFirst:
 *       Type: Bool
 *       Descr: When true, first push the ball forward a little bit to ensure it is in contact with the kicker
 *       Default: false
 *   - genevaState:
 *       Type: Int
 *       Descr: The state in which the Geneva Drive should go to. -2 means leftmost state, 0 means 'neutral' state, 2 means rightmost state.

 */
class GenevaKick : public Skill {
public:
	GenevaKick(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
	void Initialize() override;
	Status Update() override;

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        params["genevaState"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() override { return "GenevaKick"; }
private:
	int robotID;
    int genevaState;

    Vector2 ballStartPos;
    Vector2 oldBallVel;
    bool goneForward;
    time_point waitStart;

    int cycleCounter;

    // void doKick();
    // void goForward();
};


} // rtt
