#include "ros/ros.h"
#include "roboteam_tactics/skills/StandFree.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/Cone.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_msgs/WorldRobot.h"
#include <vector>

namespace rtt {

StandFree::StandFree(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard)
        , avoidRobots(n, "", private_bb) {

}

bt::Node::Status StandFree::Update() {
	// Get world and blackboard information
	roboteam_msgs::World world = LastWorld::get();
	int myID = GetInt("ROBOT_ID");
	int theirID = GetInt("theirID");

	roboteam_msgs::WorldRobot myRobot = world.us.at(myID);
	roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us.at(myID).pos.x, world.us.at(myID).pos.y);
	double myAngle = world.us.at(myID).angle;

	roboteam_utils::Vector2 ballPos = roboteam_utils::Vector2(world.ball.pos.x, world.ball.pos.y);

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
    	ROS_INFO("I can already see him, you n00b");
    } else {
    	ROS_INFO("Can't see him...");
    }

    std::vector<roboteam_msgs::WorldRobot> robotsInTheWay = getObstacles(myRobot, theirPos, &world, false);
    for (size_t i = 0; i < robotsInTheWay.size(); i++) {
    	if (robotsInTheWay.at(i).id == theirID) {
    		robotsInTheWay.erase(robotsInTheWay.begin()+i);
    	}
    }

    ROS_INFO_STREAM("length vector: " << robotsInTheWay.size());

    roboteam_utils::Vector2 nearestFreePos;
    for (size_t i = 0; i < robotsInTheWay.size(); i++) {
    	roboteam_utils::Vector2 robotInTheWayPos = roboteam_utils::Vector2(robotsInTheWay.at(i).pos.x, robotsInTheWay.at(i).pos.y);
    	ROS_INFO_STREAM("theirPos: " << theirPos.x << " " << theirPos.y);
    	ROS_INFO_STREAM("robotInTheWayPos: " << robotInTheWayPos.x << " " << robotInTheWayPos.y);
    	ROS_INFO_STREAM("myPos: " << myPos.x << " " << myPos.y);
    	Cone cone(theirPos, robotInTheWayPos, 0.2);
    	nearestFreePos = cone.ClosestPointOnSide(myPos);
    }

    ROS_INFO_STREAM("nearest pos: " << nearestFreePos.x << " " << nearestFreePos.y);

    private_bb->SetInt("ROBOT_ID", myID);
    private_bb->SetDouble("xGoal", nearestFreePos.x);
    private_bb->SetDouble("yGoal", nearestFreePos.y);
    private_bb->SetDouble("angleGoal", myAngle);
    
    avoidRobots.Update();
    
    return Status::Running;
}

} // rtt