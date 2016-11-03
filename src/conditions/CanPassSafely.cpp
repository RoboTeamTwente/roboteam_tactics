#include "roboteam_tactics/conditions/CanPassSafely.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

CanPassSafely::CanPassSafely(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    
}

bt::Node::Status AvoidRobots::Update (){
	roboteam_msgs::World world = LastWorld::get();
	int myID = GetInt("ROBOT_ID");
	int passToRobot = GetInt("passToRobot");
	if (myID == passToRobot) {
		ROS_WARNING("you're trying to pass to yourself, you silly you");
		return Status::Invalid;
	}

	roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us.at(myID).pos.x, world.us.at(myID).pos.y);	
	roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(world.us.at(passToRobot).pos.x, world.us.at(passToRobot).pos.y);
	roboteam_utils::Vector2 ballTrajectory = targetPos - myPos;

	for (size_t i = 0; i < world.them.size(); i++) {
		roboteam_utils::Vector2 theirPos = roboteam_utils::Vector2(world.them.at(i).pos.x, world.them.at(i).pos.y);
		roboteam_utils::Vector2 vectorToOtherRobot = theirPos - myPos;
		roboteam_utils::Vector2 projectionOnBallTrajectory = vectorToOtherRobot.dot(ballTrajectory) * ballTrajectory + myPos;
	}
	
}

} //rtt