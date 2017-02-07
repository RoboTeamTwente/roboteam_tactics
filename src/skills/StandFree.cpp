#include <vector>
#include <string>
#include <ros/ros.h>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"

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
    
    // Find out which robots are potentially blocking our view, by choosing those who are closer to the target than me
    std::vector<roboteam_utils::Vector2> robotsInTheWay;
    for (size_t i = 0; i < watchOutForTheseBots.size(); i++) {
        roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(watchOutForTheseBots.at(i).pos.x, watchOutForTheseBots.at(i).pos.y);
        if ((robotPos - targetPos).length() < (myPos-targetPos).length()*2) {
            robotsInTheWay.push_back(robotPos);
        }
    }

    // Get blackboard info about by how much we should stay clear of other robots
    double distanceFromPoint;
    if (HasDouble("distanceFromPoint")) {
        distanceFromPoint = GetDouble("distanceFromPoint");
    } else {
        distanceFromPoint = 0.4;
    }


    // Make a Cone for a robot standing between me and the target, and then see whether this cone overlaps with the cones of other robots,
    // in which case they can be merged into one bigger cone
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

    // If no robot is standing between me and the target, then don't return a cone
    return boost::none;
}

bt::Node::Status StandFree::Update() {


	// Get world and blackboard information
	roboteam_msgs::World world = LastWorld::get();
	unsigned int myID = blackboard->GetInt("ROBOT_ID");
	unsigned int theirID = GetInt("theirID");


    // Store some variables for easy access
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


    // Fill a vector containing all the robots except me and the one I'm looking at
    std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots;
    for (size_t i = 0; i < world.us.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.us.at(i);
        if (!(GetString("whichTeam") == "us" && currentRobot.id == theirID) && currentRobot.id != myID) {
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.them.at(i);
        if (!(GetString("whichTeam") == "them" && currentRobot.id == theirID)) {
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
        }
    }


    // Default result is my current position
    roboteam_utils::Vector2 nearestFreePos = myPos;


    // Make a Cover Cone for the robots standing between me and the target
    boost::optional<Cone> coneRobots = MakeCoverCone(watchOutForTheseBots, myPos, theirPos);

    // If the Cover Cone exists, determine the point closest to me on the edge of the cone
    if (coneRobots) {
        Cone cone = *coneRobots;
        roboteam_utils::Vector2 theirGoalPos = LastWorld::get_their_goal_center();
        // Find the closest point to me on the side of the cone, and preferably close to their goal position
        nearestFreePos = cone.ClosestPointOnSide(myPos, theirGoalPos);

        // Draw the lines of the cone in rqt_view
        roboteam_utils::Vector2 coneSide1 = (cone.center-cone.start).rotate(0.5*cone.angle);
        roboteam_utils::Vector2 coneSide2 = (cone.center-cone.start).rotate(-0.5*cone.angle);
        drawer.DrawLine("coneRobotsSide1", cone.start, coneSide1); 
        drawer.DrawLine("coneRobotsSide2", cone.start, coneSide2);
    } else {
        drawer.RemoveLine("coneRobotsSide1"); 
        drawer.RemoveLine("coneRobotsSide2");
    }


    // Make a Cover Cone for the robots standing between me and the goal
    roboteam_utils::Vector2 goalPos = LastWorld::get_their_goal_center();
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    goalPos.y = field.goal_width / 2;
    boost::optional<Cone> coneGoal = MakeCoverCone(watchOutForTheseBots, myPos, goalPos);
    goalPos.y = -field.goal_width / 2;
    boost::optional<Cone> coneGoal2 = MakeCoverCone(watchOutForTheseBots, myPos, goalPos);


    roboteam_utils::Vector2 nearestFreePos2(100.0, 100.0);

    if (coneGoal && coneGoal2) {

        Cone cone = *coneGoal;
        Cone cone2 = *coneGoal2;
        roboteam_utils::Vector2 theirGoalPos = LastWorld::get_their_goal_center();


        // Draw the lines in rqt_view
        roboteam_utils::Vector2 cone1Side2 = (cone.center-cone.start).rotate(-0.5*cone.angle);
        cone1Side2 = cone1Side2.scale(3/cone1Side2.length());
        roboteam_utils::Vector2 cone2Side1 = (cone2.center-cone2.start).rotate(0.5*cone2.angle);
        cone2Side1 = cone2Side1.scale(3/cone2Side1.length());

        roboteam_utils::Vector2 newConeStart = Cone::LineIntersection(cone.start, cone1Side2, cone2.start, cone2Side1);
        Cone newCone(newConeStart, cone2Side1, cone1Side2);

        roboteam_utils::Vector2 newConeSide1 = (newCone.center-newCone.start).rotate(-0.5*newCone.angle);
        roboteam_utils::Vector2 newConeSide2 = (newCone.center-newCone.start).rotate(0.5*newCone.angle);

        drawer.DrawLine("newConeSide1", newCone.start, newConeSide1);
        drawer.DrawLine("newConeSide2", newCone.start, newConeSide2);

        if (coneRobots) {
            std::vector<std::string> names;
            names.push_back("goalCone1option1");
            names.push_back("goalCone1option2");
            names.push_back("goalCone1option3");
            names.push_back("goalCone1option4");
            nearestFreePos = newCone.ClosestPointOnSideTwoCones(*coneRobots, myPos, theirGoalPos, drawer, names);
        } else {
            nearestFreePos = newCone.ClosestPointOnSide(myPos, theirGoalPos);
        }        
    } else {
        drawer.RemoveLine("newConeSide1");
        drawer.RemoveLine("newConeSide2");
    }


    if ((nearestFreePos2 - myPos).length() < (nearestFreePos - myPos).length()) {
        nearestFreePos = nearestFreePos2;
    }

    drawer.SetColor(255,0,0);
    drawer.DrawPoint("nearestFreePos", nearestFreePos);
    drawer.SetColor(0,0,0);


    // Fill the goToPos blackboard and send the command
    double angleGoal = (theirPos-myPos).angle();
    private_bb->SetInt("ROBOT_ID", myID);
    private_bb->SetDouble("xGoal", nearestFreePos.x);
    private_bb->SetDouble("yGoal", nearestFreePos.y);
    private_bb->SetDouble("angleGoal", angleGoal);
    private_bb->SetBool("avoidRobots", true);
    if (goToPos.Update() == Status::Success) {
        
        // Remove all the lines from rqt view
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
