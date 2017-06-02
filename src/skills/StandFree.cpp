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
        , goToPos("", private_bb) {}


// Make a cone around any number of overlapping robots standing between myPos and targetPos, starting from targetPos
boost::optional<Cone> StandFree::MakeCoverCone(std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots, Vector2 myPos, Vector2 targetPos) {
    
    // Find out which robots are potentially blocking our view, by choosing those who are closer to the target than me
    std::vector<Vector2> robotsInTheWay;
    for (size_t i = 0; i < watchOutForTheseBots.size(); i++) {
        Vector2 robotPos = Vector2(watchOutForTheseBots.at(i).pos.x, watchOutForTheseBots.at(i).pos.y);
        if ((robotPos - targetPos).length() < (myPos-targetPos).length()) {
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
    Vector2 myPos = Vector2(getWorldBot(myID)->pos);
    Vector2 theirPos;
    
    if (HasBool("ourTeam")) {
        if (GetBool("ourTeam")) {
            theirPos = Vector2(getWorldBot(theirID)->pos); 
        } else {
            theirPos = Vector2(getWorldBot(theirID, false)->pos); 
        } 
    } else {
        theirPos = Vector2(getWorldBot(theirID)->pos); 
    }

    if (blackboard->HasDouble("xGoal") && blackboard->HasDouble("yGoal")) {
        targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    } else {
        targetPos = myPos;
    }

    Vector2 closeTo;
    if (blackboard->HasDouble("closeToX") && blackboard->HasDouble("closeToY")) {
        closeTo = Vector2(GetDouble("closeToX"), GetDouble("closeToY"));
    } else {
        closeTo = myPos;
    }
 
    // Fill a vector containing all the robots except me and the one I'm looking at
    std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots;
    for (size_t i = 0; i < world.us.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.us.at(i);
        if (!(GetBool("ourTeam") && currentRobot.id == theirID) && currentRobot.id != myID) {
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.them.at(i);
        if (!(!GetBool("ourTeam") && currentRobot.id == theirID)) {
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
        }
    }

    // Default result is the target position
    Vector2 nearestFreePos = targetPos;

    // Make a Cover Cone for the robots standing between me and the target
    boost::optional<Cone> coneRobots = MakeCoverCone(watchOutForTheseBots, targetPos, theirPos);

    // If the Cover Cone exists, determine the point closest to me on the edge of the cone
    if (coneRobots) {
        Cone cone = *coneRobots;

        // Find the closest point to me on the side of the cone, and preferably close to the closeTo point
        nearestFreePos = cone.ClosestPointOnSide(targetPos, closeTo);

        // Draw the lines of the cone in rqt_view
        Vector2 coneSide1 = (cone.center-cone.start).rotate(0.5*cone.angle);
        Vector2 coneSide2 = (cone.center-cone.start).rotate(-0.5*cone.angle);
        drawer.drawLine("coneRobotsSide1", cone.start, coneSide1.scale(3)); 
        drawer.drawLine("coneRobotsSide2", cone.start, coneSide2.scale(3));
    } else {
        drawer.removeLine("coneRobotsSide1"); 
        drawer.removeLine("coneRobotsSide2");
    }

    if (HasBool("seeGoal") && GetBool("seeGoal")) {
        // Make a Cover Cone for the robots standing between me and the goal
        Vector2 goalPos = LastWorld::get_their_goal_center();
        roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
        goalPos.y = field.goal_width / 2;
        boost::optional<Cone> coneGoal = MakeCoverCone(watchOutForTheseBots, targetPos, goalPos);
        goalPos.y = -field.goal_width / 2;
        boost::optional<Cone> coneGoal2 = MakeCoverCone(watchOutForTheseBots, targetPos, goalPos);

        if (coneGoal && coneGoal2) {

            Cone cone = *coneGoal;
            Cone cone2 = *coneGoal2;

            // Draw the lines in rqt_view
            Vector2 cone1Side2 = (cone.center-cone.start).rotate(-0.5*cone.angle);
            cone1Side2 = cone1Side2.scale(3/cone1Side2.length());
            Vector2 cone2Side1 = (cone2.center-cone2.start).rotate(0.5*cone2.angle);
            cone2Side1 = cone2Side1.scale(3/cone2Side1.length());

            Vector2 newConeStart = Cone::LineIntersection(cone.start, cone1Side2, cone2.start, cone2Side1);
            Cone newCone(newConeStart, cone2Side1, cone1Side2);

            Vector2 newConeSide1 = (newCone.center-newCone.start).rotate(-0.5*newCone.angle);
            Vector2 newConeSide2 = (newCone.center-newCone.start).rotate(0.5*newCone.angle);

            drawer.setColor(255, 255, 0);
            drawer.drawLine("newConeSide1", cone.start, newConeSide1.scale(3));
            drawer.drawLine("newConeSide2", cone2.start, newConeSide2.scale(3));
            drawer.setColor(0, 0, 0);

            if (coneRobots) {
                std::vector<std::string> names;
                names.push_back("goalCone1option1");
                names.push_back("goalCone1option2");
                names.push_back("goalCone1option3");
                names.push_back("goalCone1option4");
                nearestFreePos = newCone.ClosestPointOnSideTwoCones(*coneRobots, nearestFreePos, closeTo, drawer, names);
            } else {
                nearestFreePos = newCone.ClosestPointOnSide(nearestFreePos, closeTo);
            }        
        } else {
            drawer.removeLine("newConeSide1");
            drawer.removeLine("newConeSide2");
        }
    }

    drawer.setColor(255,0,0);
    drawer.drawPoint("nearestFreePos", nearestFreePos);
    drawer.setColor(0,0,0);

    // Fill the goToPos blackboard and send the command
    double angleGoal = (theirPos-targetPos).angle();
    private_bb->SetInt("ROBOT_ID", myID);
    private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));
    private_bb->SetDouble("xGoal", nearestFreePos.x);
    private_bb->SetDouble("yGoal", nearestFreePos.y);
    private_bb->SetDouble("angleGoal", angleGoal);
    private_bb->SetBool("avoidRobots", true);
    private_bb->SetBool("avoidDefenseAreas", GetBool("avoidDefenseAreas", true));
    if (goToPos.Update() == Status::Success) {
        // Remove all the lines from rqt view
        drawer.removeLine("coneRobotsSide1");
        drawer.removeLine("coneRobotsSide2");
        drawer.removeLine("newConeSide1");
        drawer.removeLine("newConeSide2");
        drawer.removePoint("nearestFreePos");
        return Status::Success;
    }
    return Status::Running;
}

} // rtt
