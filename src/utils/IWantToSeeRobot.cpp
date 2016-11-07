#include "roboteam_tactics/conditions/IWantToSeeRobot.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

IWantToSeeRobot::IWantToSeeRobot(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    
}

bt::Node::Status IWantToSeeRobot::Update() {
	// Get world and blackboard information
	roboteam_msgs::World world = LastWorld::get();
	int myID = GetInt("ROBOT_ID");
	int theirID = GetInt("theirID");

	roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us.at(myID).pos.x, world.us.at(myID).pos.x);
	double myAngle = world.us.at(myID).angle;

	roboteam_utils::Vector2 theirPos;
	double theirAngle;
	if (GetString("whichTeam") == "us") {
		theirPos = roboteam_utils::Vector2(world.us.at(theirID).pos.x, world.us.at(theirID).pos.y); 
		theirAngle = world.us.at(theirID).angle;
	} else if (GetString("whichTeam") == "them") {
		theirPos = roboteam_utils::Vector2(world.them.at(theirID).pos.x, world.them.at(theirID).pos.y); 
		theirAngle = world.them.at(theirID).angle;
	} else {
		ROS_WARN("No team specified...");
	}

	auto bb2 = std::make_shared<bt::Blackboard>();
    bb2->SetInt("me", myID);
    bb2->SetDouble("x_coor", theirPos.x);
    bb2->SetDouble("y_coor", theirPos.y);
    bb2->SetBool("check_move", true);
	CanSeePoint canSeePoint("", bb2);
    if (canSeePoint.Update() == Status::Success) {
    	// I can already see him
    	
    }
	
}