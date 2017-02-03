#include <vector>
#include <string>
#include <ros/ros.h>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Cone.h"

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/skills/StandFree.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"

namespace rtt {

RTT_REGISTER_SKILL(StandFree);

StandFree::StandFree(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPos("", private_bb) {
            ROS_INFO_STREAM("Standing Free");
}

boost::optional<Cone> StandFree::MakeCoverCone(std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots, roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 targetPos) {
    // Find out which of these robots are standing between me and the one I'm looking at
    std::vector<roboteam_utils::Vector2> robotsInTheWay;
    for (size_t i = 0; i < watchOutForTheseBots.size(); i++) {
        roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(watchOutForTheseBots.at(i).pos.x, watchOutForTheseBots.at(i).pos.y);
        if ((robotPos - targetPos).length() < (myPos-targetPos).length()) {
            robotsInTheWay.push_back(robotPos);
        }
    }

    // Make a Cone for a robot standing between me and the target, and then see whether this cone overlaps with the cones of other robots,
    // in which case they can be merged into a big cone
    double distanceFromPoint;
    if (HasDouble("distanceFromPoint")) {
        distanceFromPoint = GetDouble("distanceFromPoint");
    } else {
        distanceFromPoint = 0.4;
    }
    
    for (size_t i = 0; i < robotsInTheWay.size(); i++) {
        Cone cone(targetPos, robotsInTheWay.at(i), distanceFromPoint);
        if (cone.IsWithinCone(myPos)) {
            for (size_t j = 0; j < robotsInTheWay.size(); j++) {
                if (i!=j) {
                    Cone cone2(targetPos, robotsInTheWay.at(j), distanceFromPoint);
                    if (cone.DoConesOverlap(cone2)) {
                        cone = cone.MergeCones(cone2);
                    }
                }
            }
            return cone;
        }
    }
    return boost::none;
}

bt::Node::Status StandFree::Update() {
	// Get world and blackboard information
	roboteam_msgs::World world = LastWorld::get();
	unsigned int myID = blackboard->GetInt("ROBOT_ID");
	unsigned int theirID = GetInt("theirID");
    bool setRosParam = GetBool("setRosParam");

	roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us.at(myID).pos);
	roboteam_utils::Vector2 theirPos;
	if (GetString("whichTeam") == "us") {
		theirPos = roboteam_utils::Vector2(world.us.at(theirID).pos); 
	} else if (GetString("whichTeam") == "them") {
		theirPos = roboteam_utils::Vector2(world.them.at(theirID).pos); 
	} else {
		ROS_WARN("No team specified...");
        theirPos = roboteam_utils::Vector2(world.us.at(theirID).pos); 
	}

	auto bb2 = std::make_shared<bt::Blackboard>();
    bb2->SetInt("me", myID);
    bb2->SetDouble("x_coor", theirPos.x);
    bb2->SetDouble("y_coor", theirPos.y);
    bb2->SetBool("check_move", true);
	CanSeePoint canSeePoint("", bb2);

    // Fill a vector containing all the robots except me and the one I'm looking at
    std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots;
    for (size_t i = 0; i < world.us.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.us.at(i);
        if (!(GetString("whichTeam") == "us" && currentRobot.id == theirID) && currentRobot.id != myID) {
            // ROS_INFO_STREAM("watch out for " << i << " of our team!");
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.them.at(i);
        if (!(GetString("whichTeam") == "them" && currentRobot.id == theirID)) {
            // ROS_INFO_STREAM("watch out for " << i << " of their team!");
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
        }
    }
    roboteam_utils::Vector2 nearestFreePos = myPos;

    // Make a Cover Cone for the robots standing between me and the target
    boost::optional<Cone> coneRobots = MakeCoverCone(watchOutForTheseBots, myPos, theirPos);
    if (coneRobots) {
        Cone cone = *coneRobots;
        // nearestFreePos = cone.ClosestPointOnSide(myPos);
        roboteam_utils::Vector2 theirGoalPos = LastWorld::get_their_goal_center();
        nearestFreePos = cone.ClosestPointOnSide(myPos, theirGoalPos);

        // Draw the lines of the cone in rqt_view
        roboteam_utils::Vector2 coneSide1 = (cone.center-cone.start).rotate(0.5*cone.angle);
        coneSide1 = coneSide1.scale(3/coneSide1.length());
        roboteam_utils::Vector2 coneSide2 = (cone.center-cone.start).rotate(-0.5*cone.angle);
        coneSide2 = coneSide2.scale(3/coneSide2.length());
        drawer.DrawLine("coneRobotsSide1", cone.start, coneSide1);
        drawer.DrawLine("coneRobotsSide2", cone.start, coneSide2);
    } else {
        drawer.RemoveLine("coneRobotsSide1");
        drawer.RemoveLine("coneRobotsSide2");
    }

    // Make a Cover Cone for the robots standing between me and the goal
    roboteam_utils::Vector2 goalPos = LastWorld::get_their_goal_center();
    goalPos.y = 0.35;
    boost::optional<Cone> coneGoal = MakeCoverCone(watchOutForTheseBots, myPos, goalPos);
    goalPos.y = -0.35;
    boost::optional<Cone> coneGoal2 = MakeCoverCone(watchOutForTheseBots, myPos, goalPos);
    roboteam_utils::Vector2 nearestFreePos2(100.0, 100.0);

    if (coneGoal && coneGoal2) {
        Cone cone = *coneGoal;
        roboteam_utils::Vector2 theirGoalPos = LastWorld::get_their_goal_center();
        if (coneRobots) {
            nearestFreePos = cone.ClosestPointOnSideTwoCones(*coneRobots, myPos, theirGoalPos);
        } else {
            nearestFreePos = cone.ClosestPointOnSide(myPos, theirGoalPos);
        }

        Cone cone2 = *coneGoal2;
        if (coneRobots) {
            nearestFreePos2 = cone2.ClosestPointOnSideTwoCones(*coneRobots, myPos, theirGoalPos);
        } else {
            nearestFreePos2 = cone2.ClosestPointOnSide(myPos, theirGoalPos);
        }

        // Draw the lines in rqt_view
        roboteam_utils::Vector2 cone1Side1 = (cone.center-cone.start).rotate(cone.angle);
        cone1Side1 = cone1Side1.scale(3/cone1Side1.length());
        roboteam_utils::Vector2 cone2Side2 = (cone2.center-cone2.start).rotate(-cone2.angle);
        cone2Side2 = cone2Side2.scale(3/cone2Side2.length());
        drawer.DrawLine("coneGoal1Side1", cone.start, cone1Side1);
        drawer.DrawLine("coneGoal2Side2", cone2.start, cone2Side2);
    } else {
        drawer.RemoveLine("coneGoalSide1");
        drawer.RemoveLine("coneGoalSide2");
    }

    if ((nearestFreePos2 - myPos).length() < (nearestFreePos - myPos).length()) {
        nearestFreePos = nearestFreePos2;
    }

    // drawer.DrawPoint("nearestFreePos", nearestFreePos);

    // kickingTheBall is here to communicate with another skill that passes the ball towards this robot. This robot 
    // will only finish this skill once kickingTheBall is set to true by the other robot
    bool kickingTheBall;
    if (setRosParam) {
        get_PARAM_KICKING(kickingTheBall);
    } else {
        kickingTheBall = true;
    }

    double angleGoal = (theirPos-myPos).angle();
    private_bb->SetInt("ROBOT_ID", myID);
    private_bb->SetDouble("xGoal", nearestFreePos.x);
    private_bb->SetDouble("yGoal", nearestFreePos.y);
    private_bb->SetDouble("angleGoal", angleGoal);
    private_bb->SetBool("avoidRobots", true);
    if (goToPos.Update() == Status::Success && kickingTheBall) {
        // Remove the lines from rqt view
        drawer.RemoveLine("coneRobotsSide1");
        drawer.RemoveLine("coneRobotsSide2");
        drawer.RemoveLine("coneGoal1Side1");
        drawer.RemoveLine("coneGoal1Side2");
        drawer.RemoveLine("coneGoal2Side1");
        drawer.RemoveLine("coneGoal2Side2");
        drawer.RemovePoint("nearestFreePos");
        return Status::Success;
    }
    return Status::Running;
}

} // rtt
