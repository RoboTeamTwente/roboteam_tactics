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
            ROS_INFO_STREAM("Standing Free");
}

bt::Node::Status StandFree::Update() {
	// Get world and blackboard information
	roboteam_msgs::World world = LastWorld::get();
	int myID = blackboard->GetInt("ROBOT_ID");
	int theirID = GetInt("theirID");
	double distanceFromPoint = GetDouble("distanceFromPoint");
    bool setRosParam = GetBool("setRosParam");

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

    auto robotsBothTeams = world.us;
    robotsBothTeams.insert(robotsBothTeams.end(), world.them.begin(), world.them.end());

    std::vector<roboteam_utils::Vector2> robotsInTheWay;
    for (size_t i = 0; i < robotsBothTeams.size(); i++) {
        roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robotsBothTeams.at(i).pos.x, robotsBothTeams.at(i).pos.y);
        if ((robotPos - theirPos).length() < (myPos-theirPos).length()) {
            robotsInTheWay.push_back(robotPos);
        }
    }

    roboteam_utils::Vector2 nearestFreePos = myPos;
    for (int i = 0; i < robotsInTheWay.size(); i++) {
        // ROS_INFO_STREAM("i: " << i);
        Cone cone(theirPos, robotsInTheWay.at(i), distanceFromPoint);
        // ROS_INFO_STREAM("IsWithinCone " << i << ": " << cone.IsWithinCone(myPos));
        if (cone.IsWithinCone(myPos)) {
            // ROS_INFO_STREAM("ok, number " << i << " is in the way");
            for (int j = 0; j < robotsInTheWay.size(); j++) {
                // ROS_INFO_STREAM("j: " << j);
                if (i!=j) {
                    Cone cone2(theirPos, robotsInTheWay.at(j), distanceFromPoint);
                    if (cone.DoConesOverlap(cone2)) {
                        // ROS_INFO_STREAM("overlap!");
                        cone = cone.MergeCones(cone2);
                        // robotsInTheWay.erase(robotsInTheWay.begin()+j);
                        // ROS_INFO_STREAM("merged " << i << " with " << j);
                    }
                }
            }
            // ROS_INFO_STREAM("done merging");
            nearestFreePos = cone.ClosestPointOnSide(myPos);
            break;
        }
    }

    // ROS_INFO_STREAM("myPos: " << myPos.x << " " << myPos.y);
    // ROS_INFO_STREAM("nearestFreePos: " << nearestFreePos.x << " " << nearestFreePos.y);

    bool kickingTheBall;
    if (setRosParam) {
        n.getParam("/kickingTheBall", kickingTheBall); 
    } else {
        kickingTheBall = true;
    }

    if (nearestFreePos == myPos && kickingTheBall) {
        return Status::Success;
    }

    double angleGoal = (theirPos-myPos).angle();
    private_bb->SetInt("ROBOT_ID", myID);
    private_bb->SetDouble("xGoal", nearestFreePos.x);
    private_bb->SetDouble("yGoal", nearestFreePos.y);
    private_bb->SetDouble("angleGoal", angleGoal);
    avoidRobots.Update();
    
    return Status::Running;
}

} // rtt