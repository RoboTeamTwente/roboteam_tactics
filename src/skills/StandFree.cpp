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

namespace rtt {

StandFree::StandFree(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard)
        , avoidRobots(n, "", private_bb) {
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

    double distanceFromPoint = GetDouble("distanceFromPoint");
    for (int i = 0; i < robotsInTheWay.size(); i++) {
        // ROS_INFO_STREAM("i: " << i);
        Cone cone(targetPos, robotsInTheWay.at(i), distanceFromPoint);
        // ROS_INFO_STREAM("IsWithinCone " << i << ": " << cone.IsWithinCone(myPos));
        if (cone.IsWithinCone(myPos)) {
            // ROS_INFO_STREAM("ok, number " << i << " is in the way");
            for (int j = 0; j < robotsInTheWay.size(); j++) {
                // ROS_INFO_STREAM("j: " << j);
                if (i!=j) {
                    Cone cone2(targetPos, robotsInTheWay.at(j), distanceFromPoint);
                    if (cone.DoConesOverlap(cone2)) {
                        // ROS_INFO_STREAM("overlap!");
                        cone = cone.MergeCones(cone2);
                        // ROS_INFO_STREAM("merged " << i << " with " << j);
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

    // Fill a vector containing all the robots except me and the one I'm looking at
    std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots;
    for (size_t i = 0; i < world.us.size(); i++) {
        if (!(GetString("whichTeam") == "us" && i == theirID) && i != myID) {
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), world.us.at(i));
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        if (!(GetString("whichTeam") == "them" && i == theirID)) {
            watchOutForTheseBots.insert(watchOutForTheseBots.end(), world.them.at(i));
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
        coneSide1 = coneSide1.scale(1/coneSide1.length());
        roboteam_utils::Vector2 coneSide2 = (cone.center-cone.start).rotate(-cone.angle);
        coneSide2 = coneSide2.scale(1/coneSide2.length());

        roboteam_msgs::DebugLine firstLine;
        firstLine.name = "firstLine";
        roboteam_msgs::Vector2f startLine1;
        startLine1.x = cone.start.x;
        startLine1.y = cone.start.y;
        roboteam_msgs::Vector2f endLine1;
        endLine1.x = coneSide1.x + cone.start.x;
        endLine1.y = coneSide1.y + cone.start.y;
        firstLine.points.push_back(startLine1);
        firstLine.points.push_back(endLine1);
        debugPub.publish(firstLine);

        roboteam_msgs::DebugLine secondLine;
        secondLine.name = "secondLine";
        roboteam_msgs::Vector2f startLine2;
        startLine2.x = cone.start.x;
        startLine2.y = cone.start.y;
        roboteam_msgs::Vector2f endLine2;
        endLine2.x = coneSide2.x + cone.start.x;
        endLine2.y = coneSide2.y + cone.start.y;
        secondLine.points.push_back(startLine2);
        secondLine.points.push_back(endLine2);
        debugPub.publish(secondLine);
    }

    // Make a Cover Cone for the robots standing between me and the goal
    roboteam_utils::Vector2 goalPos(-3.0, 0.0);
    boost::optional<Cone> coneGoal = MakeCoverCone(watchOutForTheseBots, myPos, goalPos);
    if (coneGoal) {
        Cone cone = *coneGoal;
        
        if (coneRobots) {
            nearestFreePos = cone.ClosestPointOnSideTwoCones(*coneRobots, myPos);
            
        } else {
            nearestFreePos = cone.ClosestPointOnSide(myPos);
        }
        

        // Draw the lines of the cone in rqt_view
        roboteam_utils::Vector2 coneSide1 = (cone.center-cone.start).rotate(cone.angle);
        coneSide1 = coneSide1.scale(1/coneSide1.length());
        roboteam_utils::Vector2 coneSide2 = (cone.center-cone.start).rotate(-cone.angle);
        coneSide2 = coneSide2.scale(1/coneSide2.length());

        roboteam_msgs::DebugLine firstLine;
        firstLine.name = "thirdLine";
        roboteam_msgs::Vector2f startLine1;
        startLine1.x = cone.start.x;
        startLine1.y = cone.start.y;
        roboteam_msgs::Vector2f endLine1;
        endLine1.x = coneSide1.x + cone.start.x;
        endLine1.y = coneSide1.y + cone.start.y;
        firstLine.points.push_back(startLine1);
        firstLine.points.push_back(endLine1);
        debugPub.publish(firstLine);

        roboteam_msgs::DebugLine secondLine;
        secondLine.name = "fourthLine";
        roboteam_msgs::Vector2f startLine2;
        startLine2.x = cone.start.x;
        startLine2.y = cone.start.y;
        roboteam_msgs::Vector2f endLine2;
        endLine2.x = coneSide2.x + cone.start.x;
        endLine2.y = coneSide2.y + cone.start.y;
        secondLine.points.push_back(startLine2);
        secondLine.points.push_back(endLine2);
        debugPub.publish(secondLine);
    }

    roboteam_msgs::DebugPoint targetPosition;
    targetPosition.name = "targetPosition";
    targetPosition.pos.x = nearestFreePos.x;
    targetPosition.pos.y = nearestFreePos.y;
    debugPubPoint.publish(targetPosition);

    // // Find out which of these robots are standing between me and the one I'm looking at
    // std::vector<roboteam_utils::Vector2> robotsInTheWay;
    // for (size_t i = 0; i < robotsBothTeams.size(); i++) {
    //     roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robotsBothTeams.at(i).pos.x, robotsBothTeams.at(i).pos.y);
    //     if ((robotPos - theirPos).length() < (myPos-theirPos).length()) {
    //         robotsInTheWay.push_back(robotPos);
    //         // ROS_INFO_STREAM("robot " << i << " has ID " << robotsBothTeams.at(i).id);
    //     }
    // }

    // // Find out which of these robots are standing between me and the goal
    // roboteam_utils::Vector2 goalPos(-3.0, 0.0);
    // std::vector<roboteam_utils::Vector2> robotsInTheWayOfGoal;
    // for (size_t i = 0; i < robotsBothTeams.size(); i++) {
    //     roboteam_utils::Vector2 robotPos = roboteam_utils::Vector2(robotsBothTeams.at(i).pos.x, robotsBothTeams.at(i).pos.y);
    //     if ((robotPos - goalPos).length() < (myPos-goalPos).length()) {
    //         robotsInTheWayOfGoal.push_back(robotPos);
    //     }
    // }


    // boost::optional<Cone> a = MakeCoverCone();
    // if (a) {
    //     Cone cone = *a;
    // } else {

    // }



    // // ROS_INFO_STREAM("number of robots in the way: " << robotsInTheWay.size());
    // roboteam_utils::Vector2 nearestFreePos = myPos;
    // for (int i = 0; i < robotsInTheWay.size(); i++) {
    //     // ROS_INFO_STREAM("i: " << i);
    //     Cone cone(theirPos, robotsInTheWay.at(i), distanceFromPoint);
    //     // ROS_INFO_STREAM("IsWithinCone " << i << ": " << cone.IsWithinCone(myPos));
    //     if (cone.IsWithinCone(myPos)) {
    //         // ROS_INFO_STREAM("ok, number " << i << " is in the way");
    //         for (int j = 0; j < robotsInTheWay.size(); j++) {
    //             // ROS_INFO_STREAM("j: " << j);
    //             if (i!=j) {
    //                 Cone cone2(theirPos, robotsInTheWay.at(j), distanceFromPoint);
    //                 if (cone.DoConesOverlap(cone2)) {
    //                     // ROS_INFO_STREAM("overlap!");
    //                     cone = cone.MergeCones(cone2);
    //                     // ROS_INFO_STREAM("merged " << i << " with " << j);
    //                 }
    //             }
    //         }




    //         // ROS_INFO_STREAM("done merging");
    //         nearestFreePos = cone.ClosestPointOnSide(myPos);

    //         roboteam_utils::Vector2 coneSide1 = (cone.center-cone.start).rotate(cone.angle);
    //         coneSide1 = coneSide1.scale(1/coneSide1.length());
    //         roboteam_utils::Vector2 coneSide2 = (cone.center-cone.start).rotate(-cone.angle);
    //         coneSide2 = coneSide2.scale(1/coneSide2.length());
            
    //         // Draw the lines of the cone in rqt_view
    //         roboteam_msgs::DebugLine firstLine;
    //         firstLine.name = "firstLine";
    //         roboteam_msgs::Vector2f startLine1;
    //         startLine1.x = cone.start.x;
    //         startLine1.y = cone.start.y;
    //         roboteam_msgs::Vector2f endLine1;
    //         endLine1.x = coneSide1.x + cone.start.x;
    //         endLine1.y = coneSide1.y + cone.start.y;
    //         firstLine.points.push_back(startLine1);
    //         firstLine.points.push_back(endLine1);
    //         debugPub.publish(firstLine);

    //         roboteam_msgs::DebugLine secondLine;
    //         secondLine.name = "secondLine";
    //         roboteam_msgs::Vector2f startLine2;
    //         startLine2.x = cone.start.x;
    //         startLine2.y = cone.start.y;
    //         roboteam_msgs::Vector2f endLine2;
    //         endLine2.x = coneSide2.x + cone.start.x;
    //         endLine2.y = coneSide2.y + cone.start.y;
    //         secondLine.points.push_back(startLine2);
    //         secondLine.points.push_back(endLine2);
    //         debugPub.publish(secondLine);
    //         break;
    //     }
    // }


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
        return Status::Success;
    }
    
    return Status::Running;
}

} // rtt