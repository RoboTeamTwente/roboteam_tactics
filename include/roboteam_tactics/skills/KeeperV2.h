#pragma once

#include "ros/ros.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/ReceiveBall.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Arc.h"
#include "roboteam_utils/Draw.h"

namespace rtt {

/**
 * \class KeeperV2
 * \brief See YAML
 */
/*
 * Descr: Brand new keeper skill by Jelle
 * Params:
 *   - ROBOT_ID:
 *       Type: Int
 *       Descr: The id of the robot
 */
class KeeperV2 : public Skill {
public:
	KeeperV2(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

	void Initialize();
	Vector2 computeBlockPoint(Vector2 defendPos);
	Vector2 computeBlockPoint2(Vector2 defendPos);

	Status Update();

    static VerificationMap required_params() {
        VerificationMap params;
        params["ROBOT_ID"] = BBArgumentType::Int;
        return params;
    }
    std::string node_name() { return "KeeperV2"; }
private:
	ReceiveBall receiveBall;

	Vector2 goalPos;
	Vector2 circCenter;
	Arc blockCircle;
	double marginFromGoal;
	double W;
	double acceptableDeviation;

	Draw drawer;

};


} // rtt
