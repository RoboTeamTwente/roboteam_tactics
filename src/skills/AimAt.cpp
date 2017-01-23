#include "roboteam_tactics/treegen/LeafRegister.h"
#include <string>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/skills/AimAt.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/utils.h"

namespace rtt {

RTT_REGISTER_SKILL(AimAt);

AimAt::AimAt(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , rotateAroundPoint("", private_bb) {
}

bt::Node::Status AimAt::Update (){

	roboteam_msgs::World world = LastWorld::get();
	bool setRosParam = GetBool("setRosParam");

	int robotID = blackboard->GetInt("ROBOT_ID");
	// if (robotID == 1) ROS_INFO_STREAM("AimAt Update"); 
	std::string destination = GetString("At");

	// Check is world contains a sensible message. Otherwise wait, it might the case that AimAt::Update 
	// is called before the first world state update
	if (world.us.size() == 0) {
		return Status::Running;
	}
	
	roboteam_utils::Vector2 passTo;
	
	if (destination=="robot"){
        int AtRobotID = GetInt("AtRobot");
        auto possibleBot = lookup_our_bot(AtRobotID);
        if (possibleBot) {
            passTo = roboteam_utils::Vector2(possibleBot->pos);
        } else {
            return Status::Failure;
        }
	} else if(destination=="theirgoal"){
        passTo = LastWorld::get_their_goal_center();
	} else if(destination=="ourgoal"){
        passTo = LastWorld::get_our_goal_center();
	}
	
    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetString("center", "ball");
    private_bb->SetDouble("faceTowardsPosx", passTo.x);
    private_bb->SetDouble("faceTowardsPosy", passTo.y);
    private_bb->SetDouble("w",3.0);
    private_bb->SetDouble("radius", 0.1);

    Status result = rotateAroundPoint.Update();
	if (result == Status::Success) {
		if (setRosParam) {
			set_PARAM_KICKING(true);
		}
		return Status::Success;
	} else if (result == Status::Failure) {
		// ROS_INFO_STREAM("AimAt failed :(");
		return Status::Failure;
	} else {
		return Status::Running;
	}
};

} // rtt
