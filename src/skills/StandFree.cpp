#include "ros/ros.h"
#include "roboteam_tactics/skills/StandFree.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/Cone.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/DebugPoint.h"
#include "roboteam_msgs/DebugLine.h"
#include <vector>
#include <string>

namespace rtt {

StandFree::StandFree(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , avoidRobots("", private_bb) {
            ROS_INFO_STREAM("Standing Free");
            debugPub = n.advertise<roboteam_msgs::DebugLine>("view_debug_lines", 1000);
            debugPubPoint = n.advertise<roboteam_msgs::DebugPoint>("view_debug_points", 1000);
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
    double distanceFromPoint = GetDouble("distanceFromPoint");
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

void StandFree::DrawLine(std::string name, roboteam_utils::Vector2 start, roboteam_utils::Vector2 line) {
    roboteam_msgs::DebugLine Line; 
    Line.name = name; 
    Line.remove = false;
    roboteam_msgs::Vector2f startLine1; 
    startLine1.x = start.x; 
    startLine1.y = start.y;
    roboteam_msgs::Vector2f endLine1;
    endLine1.x = line.x + start.x; 
    endLine1.y = line.y + start.y;
    Line.points.push_back(startLine1); 
    Line.points.push_back(endLine1);
    debugPub.publish(Line);
}

void StandFree::RemoveLine(std::string name) {
    roboteam_msgs::DebugLine Line; 
    Line.name = name; 
    Line.remove = true;
    debugPub.publish(Line);
}

void StandFree::DrawPoint(std::string name, roboteam_utils::Vector2 point) {
    roboteam_msgs::DebugPoint position; 
    position.name = name;
    position.pos.x = point.x; 
    position.pos.y = point.y;
    debugPubPoint.publish(position);
}

void StandFree::RemovePoint(std::string name) {
    roboteam_msgs::DebugPoint position; 
    position.name = name;
    position.remove = true;
    debugPubPoint.publish(position);
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
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.them.at(i);
        if (!(GetString("whichTeam") == "them" && currentRobot.id == theirID)) {
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
        }
    }
    roboteam_utils::Vector2 nearestFreePos = myPos;

    // Make a Cover Cone for the robots standing between me and the target
    boost::optional<Cone> coneRobots = MakeCoverCone(watchOutForTheseBots, myPos, theirPos);
    if (coneRobots) {
        Cone cone = *coneRobots;
        nearestFreePos = cone.ClosestPointOnSide(myPos);

        // Draw the lines of the cone in rqt_view
        roboteam_utils::Vector2 coneSide1 = (cone.center-cone.start).rotate(cone.angle);
        coneSide1 = coneSide1.scale(3/coneSide1.length());
        roboteam_utils::Vector2 coneSide2 = (cone.center-cone.start).rotate(-cone.angle);
        coneSide2 = coneSide2.scale(3/coneSide2.length());
        DrawLine("coneRobotsSide1", cone.start, coneSide1);
        DrawLine("coneRobotsSide2", cone.start, coneSide2);
    } else {
        RemoveLine("coneRobotsSide1");
        RemoveLine("coneRobotsSide2");
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
        if (coneRobots) {
            nearestFreePos = cone.ClosestPointOnSideTwoCones(*coneRobots, myPos);
        } else {
            nearestFreePos = cone.ClosestPointOnSide(myPos);
        }

        Cone cone2 = *coneGoal2;
        if (coneRobots) {
            nearestFreePos2 = cone2.ClosestPointOnSideTwoCones(*coneRobots, myPos);
        } else {
            nearestFreePos2 = cone2.ClosestPointOnSide(myPos);
        }

        // Draw the lines in rqt_view
        roboteam_utils::Vector2 coneSide1 = (cone.center-cone.start).rotate(cone.angle);
        coneSide1 = coneSide1.scale(3/coneSide1.length());
        roboteam_utils::Vector2 coneSide2 = (cone2.center-cone2.start).rotate(-cone2.angle);
        coneSide2 = coneSide2.scale(3/coneSide2.length());
        DrawLine("coneGoalSide1", cone.start, coneSide1);
        DrawLine("coneGoalSide2", cone2.start, coneSide2);
    } else {
        RemoveLine("coneGoalSide1");
        RemoveLine("coneGoalSide2");
    }

    if ((nearestFreePos2 - myPos).length() < (nearestFreePos - myPos).length()) {
        nearestFreePos = nearestFreePos2;
    }

    DrawPoint("nearestFreePos", nearestFreePos);

    // kickingTheBall is here to communicate with another skill that passes the ball towards this robot. This robot 
    // will only finish this skill once kickingTheBall is set to true by the other robot
    bool kickingTheBall;
    if (setRosParam) {
        n.getParam("/kickingTheBall", kickingTheBall); 
    } else {
        kickingTheBall = true;
    }

    double angleGoal = (theirPos-myPos).angle();
    private_bb->SetInt("ROBOT_ID", myID);
    private_bb->SetDouble("xGoal", nearestFreePos.x);
    private_bb->SetDouble("yGoal", nearestFreePos.y);
    private_bb->SetDouble("angleGoal", angleGoal);
    if (avoidRobots.Update() == Status::Success && kickingTheBall) {
        // Remove the lines from rqt view
        RemoveLine("coneRobotsSide1");
        RemoveLine("coneRobotsSide2");
        RemoveLine("coneGoalSide1");
        RemoveLine("coneGoalSide2");
        RemovePoint("nearestFreePos");
        return Status::Success;
    }
    return Status::Running;
}

} // rtt
