#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ros/ros.h>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/FieldLineSegment.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/conditions/IsInDefenseArea.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Section.h"

#define RTT_CURRENT_DEBUG_TAG GoToPos
#define ROS_LOG_NAME "skills.GoToPos"

namespace rtt {

RTT_REGISTER_SKILL(GoToPos);

GoToPos::GoToPos(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)

        {
            succeeded = false;
            failure = false;
            controller.Initialize(blackboard->GetInt("ROBOT_ID"));

            // Debug info
            ros::NodeHandle n;
            myPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myPosTopic", 1000);
            myVelTopic = n.advertise<roboteam_msgs::WorldRobot>("myVelTopic", 1000);
            myTargetPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myTargetPosTopic", 1000);

            //DEFAULTS
            ros::param::get("robot_output_target", robot_output_target);
            if (robot_output_target == "grsim") {
                grsim = true;
                safetyMarginGoalAreas = 0.1;
                marginOutsideField = 0.3;
                avoidRobotsGain = 1.0;
                cushionGain = 1.0;
                maxDistToAntenna = 0.3; // no force is exerted when distToAntenna is larger than maxDistToAntenna
            } else {
                grsim = false;
                safetyMarginGoalAreas = 0.1;
                marginOutsideField = 0.3;
                avoidRobotsGain = 1.0;
                cushionGain = 1.5;
                maxDistToAntenna = 0.3; // no force is exerted when distToAntenna is larger than maxDistToAntenna
            }

            //PROCESS BLACKBOARD
            if (HasDouble("avoidRobotsGain")) {
                avoidRobotsGain = GetDouble("avoidRobotsGain");
            }
            if (HasDouble("cushionGain")) {
                cushionGain = GetDouble("cushionGain");
            }
            if (HasDouble("maxDistToAntenna")) {
                maxDistToAntenna = GetDouble("maxDistToAntenna");
            }
        }

void GoToPos::Initialize() {
    if (GetBool("driveBackward")) {
        ROBOT_ID = blackboard->GetInt("ROBOT_ID");
        // Get the latest world state
        roboteam_msgs::World world = LastWorld::get();
        // Find the robot with the specified ID
        boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(ROBOT_ID);
        roboteam_msgs::WorldRobot me;
        if (findBot) {
            me = *findBot;
        } else {
            return;
        }

        backwardPos = Vector2(me.pos) + Vector2(-0.3,0).rotate(me.angle);
    }
}

void GoToPos::sendStopCommand(uint id) {
    roboteam_msgs::RobotCommand command;
    command.id = ROBOT_ID;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;
    command.dribbler = false;

    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}


// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them7
// Based on this algorithm: https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
// Tweaked by Jelle, to prevent division by zero and having a maximum force that can be applied (by using a minimum distance)
Vector2 GoToPos::getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna, Vector2 sumOfForces) {

    Vector2 closestPoint = antenna.closestPointOnVector(myPos, otherRobotPos);
    Vector2 meToPoint = closestPoint - myPos;
    double L = meToPoint.length(); // distance from me to closest point
    Vector2 force = closestPoint - otherRobotPos; // Vector from other robot to closest point on antenna.
    double distToAntenna = force.length(); // distance between closest point on antenna and other robot.

    // Check if the point lies within the antenna (so not on the edges) and if the point is close enough the robot.
    if (L > 0.001 && L < antenna.length()-0.001 && distToAntenna <= maxDistToAntenna){
        // Calculate the needed avoidance force.
        if(distToAntenna > 0.0005) { // avoid division by 0
            force = force.stretchToLength(avoidRobotsGain * (1-distToAntenna/maxDistToAntenna)); //* (1-L/antenna.length())
        } else { // distToAntenna almost zero -> force direction becomes 90 degree CCW rotation of antenna
            force = (antenna.rotate(M_PI/2)).stretchToLength(avoidRobotsGain ); // * (1-L/antenna.length())
        }

        // If I'm very close to a robot, the component of sumOfForces in that direction will be (partially) subtracted
        double max = antenna.length();
        double bot_margin = 0.14;
        if (L < max && sumOfForces.dot(meToPoint) > 0 && antenna.length() > 0.3) {
            Vector2 dirForce = sumOfForces.project2(meToPoint);
            double scaleDown = (1-(L-bot_margin)/(max-bot_margin));
            if (distToAntenna > 0.10) {
                scaleDown = scaleDown*(1-(distToAntenna-0.10)/(maxDistToAntenna-0.10));
            }
            scaleDown = fmin(0.8, scaleDown);
            force = force - dirForce.scale(scaleDown);
        }

    } else {
        force = Vector2(0,0);
    }


    return force;
}


// Computes a vector that can be added to sumOfForces, in order to avoid crashing into other robots
Vector2 GoToPos::avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces) {
    roboteam_msgs::World world = LastWorld::get();
    Vector2 posError = targetPos - myPos;
    // The antenna is a vector starting at the robot position in the direction in which it wants to go (scaled to the robot speed)
    double minAntenna = 1.2; // antenna has a minimal length, to make sure it still avoids when it starts from a standstill.
    Vector2 antenna = posError.stretchToLength(minAntenna + myVel.length()*0.8);
    // The antenna will not grow beyond its minimum value when moving backwards
    if (myVel.dot(antenna)<0 ){
        antenna = posError.stretchToLength(minAntenna);
    }
    // The antenna is never larger than the position error
    if (posError.length() < antenna.length()) {
            antenna = posError;
    }

    // For all robots in the field that are closer than the antenna length to our robot: ..
    // ..determine if they exert a repelling force and add all these to sumOfForces.
    // A 'cushion force' that damps out the 'crash velocity' was added by Jelle to prevent hard collisions that cant be avoided.
    // To find a better way of taking velocities into account when avoiding robots, Jelle replaced the use of estimated future positions ..
    // ..by the concept of a 'relative antenna', which is a rotated version of the antenna for each robot that is close, ..
    // ..based on the relative velocity between me and the other robot.
    int num_us = (int)world.us.size();
    int num_them = (int)world.them.size();
    for (int i = 0; i < num_us + num_them; i++) { // loop through our robots and their robots
        // extract pos and vel information of current robot
        Vector2 otherRobotPos;
        Vector2 otherRobotVel;
        if (i < num_us) {
            if (world.us.at(i).id == ROBOT_ID) {
                continue; // skip over myself
            }
            otherRobotPos = Vector2(world.us.at(i).pos);
            otherRobotVel = Vector2(world.us.at(i).vel);
        } else {
            otherRobotPos = Vector2(world.them.at(i - num_us).pos);
            otherRobotVel = Vector2(world.them.at(i - num_us).vel);
        }

        // find position of other robot wrt me
        Vector2 relativePos = otherRobotPos - myPos;
        double distToRobot = relativePos.length();

        // robots further away than my antenna are not taken into account, so we skip over these
        if (distToRobot > antenna.length()) {
            continue;
        }

        // rotate the antenna based on the relative velocity of me and the other robot
        Vector2 relativeVel = myVel - otherRobotVel;
        Vector2 relativeAntenna = antenna;
        if (relativeVel.length()>0.1 && myVel.length()>0.1) { // used velocities must be large enough to be reliable
            double rotation = cleanAngle(relativeVel.angle() - myVel.angle());
            relativeAntenna = antenna.rotate(rotation);
        // TODO: remember why did I did it like this, and not simply use relativeVel directly as direction of my antenna. Becomes untable?
        }

        // robots outside my field of view will be skipped
        if (relativeAntenna.dot(relativePos) < 0) {
            continue;
        }

        // Compute avoidance force and add to total
        Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, relativeAntenna, sumOfForces);
        sumOfForces = sumOfForces + forceVector;
        // Determine the crash velocity (velocity at which the robots would crash into eachother)..
        // ..and determine the 'cushion force' to damp out this crash velocity.
        Vector2 crashVel = relativeVel.project2(relativePos);
        Vector2 cushionForce = crashVel.scale(-cushionGain*(1-distToRobot/antenna.length()) );
        //I will not respond to a negative crashVel or a crashVel not (or barely) caused by me
        if (crashVel.dot(relativePos) < 0 || myVel.dot(relativePos) < 0.1) {
            cushionForce = Vector2(0,0);
        }
        sumOfForces = sumOfForces + cushionForce; // We add the cushion force to the total

        // draw both forces in rqt
        drawer.setColor(0, 0, 255);
        drawer.drawLine("avoidForce"+std::to_string(i),otherRobotPos,forceVector.scale(0.5));
        drawer.setColor(255, 0, 255);
        drawer.drawLine("cushionForce"+std::to_string(i),otherRobotPos,cushionForce.scale(0.5));
        drawer.setColor(155, 155, 155);
        drawer.drawPoint("relativeAntenna"+std::to_string(i),myPos+relativeAntenna);
        // }
    }



    // for (auto const currentRobot : world.us) {
    //     if (currentRobot.id != ROBOT_ID) {
    //         Vector2 otherRobotPos(currentRobot.pos);
    //         Vector2 otherRobotVel(currentRobot.vel);
    //         Vector2 relativePos = otherRobotPos - myPos;
    //         double distToRobot = relativePos.length();

    //         Vector2 relativeVel = myVel - otherRobotVel;
    //         Vector2 relativeAntenna = antenna;
    //         if (relativeVel.length()>0.2 && myVel.length()>0.1) {
    //             double rotation = cleanAngle(relativeVel.angle() - myVel.angle());
    //             relativeAntenna = antenna.rotate(rotation);
    //         }

    //         if (distToRobot <= antenna.length() && fabs(cleanAngle(relativePos.angle() - relativeAntenna.angle())) < 0.35*M_PI && relativeAntenna.dot(antenna) > 0) {
    //             // Compute avoidance force and add to total
    //             Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, relativeAntenna, sumOfForces);
    //             sumOfForces = sumOfForces + forceVector;
    //             // Determine the crash velocity (velocity at which the robots would crash into eachother)..
    //             // ..and determine the 'cushion force' to damp out this crash velocity.
    //             Vector2 crashVel = (otherRobotVel - myVel).project2(myPos - otherRobotPos);
    //             Vector2 cushionForce = crashVel.scale(cushionGain * (1-distToRobot/antenna.length()) );
    //             //I will not respond to a negative crashVel or a crashVel not (or barely) caused by me
    //             if (crashVel.dot(relativePos) > 0 || myVel.dot(relativePos) < 0.1) {
    //                 cushionForce = Vector2(0,0);
    //             }
    //             sumOfForces = sumOfForces + cushionForce; // We add the cushion force to the total
    //             // draw both forces in rqt
    //             drawer.setColor(0, 0, 255);
    //             drawer.drawLine("avoidForce" + std::to_string(currentRobot.id),otherRobotPos,forceVector.scale(0.5));
    //             drawer.setColor(255, 0, 255);
    //             drawer.drawLine("cushionForce" + std::to_string(currentRobot.id),otherRobotPos,cushionForce.scale(0.5));
    //         }
    //     }
    // }
    // for (size_t i = 0; i < world.them.size(); i++) {
    //         Vector2 otherRobotPos(world.them.at(i).pos);
    //         Vector2 otherRobotVel(world.them.at(i).vel);
    //         Vector2 relativePos = otherRobotPos - myPos;
    //         double distToRobot = relativePos.length();

    //         Vector2 relativeVel = myVel - otherRobotVel;
    //         Vector2 relativeAntenna = antenna;
    //         if (relativeVel.length()>0.2 && myVel.length()>0.1) {
    //             relativeAntenna = antenna.rotate(relativeVel.angle()-myVel.angle());
    //         }

    //         if (distToRobot <= antenna.length() && fabs(cleanAngle(relativePos.angle() - relativeAntenna.angle())) < 0.35*M_PI && relativeAntenna.dot(antenna) > 0) {
    //             // Compute avoidance force and add to total
    //             Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, relativeAntenna, sumOfForces);
    //             sumOfForces = sumOfForces + forceVector;
    //             // Determine the crash velocity (velocity at which the robots would crash into eachother)..
    //             // ..and determine the 'cushion force' to damp out this crash velocity.
    //             Vector2 crashVel = (otherRobotVel - myVel).project2(myPos - otherRobotPos);
    //             Vector2 cushionForce = crashVel.scale(cushionGain * (1-distToRobot/antenna.length()) );
    //             //I will not respond to a negative crashVel or a crashVel not (or barely) caused by me
    //             if (crashVel.dot(relativePos) > 0 || myVel.dot(relativePos) < 0.1) {
    //                 cushionForce = Vector2(0,0);
    //             }
    //             sumOfForces = sumOfForces + cushionForce; // We add the cushion force to the total
    //             // draw both forces in rqt
    //             drawer.setColor(0, 0, 255);
    //             drawer.drawLine("avoidForceThem" + std::to_string(world.them.at(i).id),otherRobotPos,forceVector.scale(0.5));
    //             drawer.setColor(255, 0, 255);
    //             drawer.drawLine("cushionForceThem" + std::to_string(world.them.at(i).id),otherRobotPos,cushionForce.scale(0.5));
    //         }
    // }

    drawer.setColor(0, 155, 155);
    drawer.drawPoint("antenna" + std::to_string(ROBOT_ID),myPos+antenna);

    return sumOfForces;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid the goal areas by moving only parralel to the goal area when close
Vector2 GoToPos::avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces) {
    Vector2 posError = targetPos - myPos;
    Vector2 newSumOfForces = sumOfForces;
    double max = 0.3;

    if (posError.length() > max) {

        if (ROBOT_ID != KEEPER_ID) {
            // Vector2 distToOurDefenseArea = getDistToDefenseArea(true, myPos, safetyMarginGoalAreas);
            Vector2 distToOurDefenseArea = getDistToDefenseArea(true, myPos, 0.0);
            double dist = distToOurDefenseArea.length();
            if (isWithinDefenseArea(true, myPos, 0.001)) {
                // If we are already in the defense area, it's best just to drive straight out of it and not to add any weird forces
            }  else if (dist < max + safetyMarginGoalAreas && dist > 0.001 && sumOfForces.dot(distToOurDefenseArea) > 0) {
                Vector2 dirForce = sumOfForces.project2(distToOurDefenseArea);
                newSumOfForces = sumOfForces - dirForce.scale(1 - (dist - safetyMarginGoalAreas)/max);
            }

        }

        Vector2 distToTheirDefenseArea = getDistToDefenseArea(false, myPos, 0.0);
        double dist = distToTheirDefenseArea.length();
        if (isWithinDefenseArea(false, myPos, 0.001)) {
            // If we are already in the defense area, it's best just to drive straight out of it and not to add any weird forces
        } else if (dist < max + safetyMarginGoalAreas && dist > 0.001 && sumOfForces.dot(distToTheirDefenseArea) > 0) {
            Vector2 dirForce = sumOfForces.project2(distToTheirDefenseArea);
            newSumOfForces = sumOfForces - dirForce.scale(1 - (dist - safetyMarginGoalAreas)/max);
        }
    }

    return newSumOfForces;
}


Vector2 GoToPos::avoidBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces, Vector2 targetPos, Vector2 myVel) {

    Vector2 posError = targetPos - myPos;

    // The antenna is a vector starting at the robot position in the direction in which it wants to go (scaled to the robot speed)
    double minAntenna = 1.2; // antenna has a minimal length, to make sure it still avoids when it starts from a standstill.
    Vector2 antenna = posError.stretchToLength(minAntenna + myVel.length()*0.8);
    // The antenna will not grow beyond its minimum value when moving backwards
    if (myVel.dot(antenna)<0 ){
        antenna = posError.stretchToLength(minAntenna);
    }
    // The antenna is never larger than the position error
    if (posError.length() < antenna.length()) {
            antenna = posError;
    }

    return getForceVectorFromRobot(myPos, ballPos, antenna, sumOfForces);
}


// Makes sure that the given target position is not inside a goal area, or (too far) outside the field. "Too far" is specified by the class variable marginOutsideField
Vector2 GoToPos::checkTargetPos(Vector2 targetPos, Vector2 myPos) {
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

    double xGoal = targetPos.x;
    double yGoal = targetPos.y;

    double newMargin = marginOutsideField;
    // In and around the goal the robot cannot cross the backline.
    if (fabs(yGoal) < (field.goal_width/2 + 0.2)) {
        newMargin = -0.1;
    }
    if (fabs(yGoal) < (field.goal_width / 2)) {
        newMargin = -0.05;
    }

    // If the target position is outside of the field + margins, then change the target position to the closest point within this margin
    // if (GetBool("ignoreFieldBounds", false)) {
        if (xGoal > (field.field_length/2+newMargin) || xGoal < (-field.field_length/2-newMargin)) {
            xGoal = signum(xGoal)*(field.field_length/2+newMargin);
        }
        if (yGoal > (field.field_width/2+marginOutsideField) || yGoal < -(field.field_width/2+marginOutsideField)) {
            yGoal = signum(yGoal)*(field.field_width/2+marginOutsideField);
        }
    // }

    Vector2 newTargetPos(xGoal, yGoal);

    // If the current robot is not a keeper, we should take into account that it cannot enter the defense area
    // if (ROBOT_ID != KEEPER_ID) {
    if (!(HasBool("enterDefenseAreas") && GetBool("enterDefenseAreas"))) {
        // If the target position is in our defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea(true, newTargetPos, safetyMarginGoalAreas)) {
            Vector2 distToOurDefenseArea = getDistToDefenseArea(true, newTargetPos, safetyMarginGoalAreas);
            drawer.setColor(50, 50, 50);
            drawer.drawLine("defAreaLine", newTargetPos, distToOurDefenseArea);
            newTargetPos = newTargetPos + distToOurDefenseArea;
        }

        // If the target position is in their defense area, then subtract the vector difference between the defense area and the target position
        if (ROBOT_ID != KEEPER_ID && isWithinDefenseArea(false, newTargetPos, safetyMarginGoalAreas)) {
            Vector2 distToTheirDefenseArea = getDistToDefenseArea(false, newTargetPos, safetyMarginGoalAreas);
            drawer.setColor(50, 50, 100);
            drawer.drawLine("defTheirAreaLine", newTargetPos, distToTheirDefenseArea);
            newTargetPos = newTargetPos + distToTheirDefenseArea;
        }
    }

    // Check if we have to stay on our side of the field
    if (HasString("stayOnSide")) {
        auto side = GetString("stayOnSide");
        auto field = LastWorld::get_field();
        double top = field.field_width / 2,
               right = field.field_length / 2 + marginOutsideField,
               bottom = field.field_width / -2,
               left = 0
               ;

        // @Hack omfg this is so f#$%& ugly
        bool isAllWhitespace = std::all_of(
                side.cbegin(),
                side.cend(),
                static_cast<int (*)(int)>(std::isspace)
                );

        if (!(side.empty() || isAllWhitespace || side == "TOTALLY EMPTY")) {
            if (side == "ourSide") {
                left = field.field_length / -2 - marginOutsideField;
                right = 0;
            } else if (side == "theirSide") {
                left = 0;
                right = field.field_length / 2 + marginOutsideField;
            }

            if (newTargetPos.y > top) {
                newTargetPos.y = top;
            }
            if (newTargetPos.x > right) {
                newTargetPos.x = right;
            }
            if (newTargetPos.y < bottom) {
                newTargetPos.y = bottom;
            }
            if (newTargetPos.x < left) {
                newTargetPos.x = left;
            }
        }
    }

    /////////////////////////
    // Check to prevent getting stuck behind the rectangular defense area
    Vector2 posError = newTargetPos - myPos;
    if (!(HasBool("enterDefenseAreas") && GetBool("enterDefenseAreas")) && posError.length() > 0.3) {
        Section posErrorSec(myPos.x, myPos.y, newTargetPos.x, newTargetPos.y);
        //roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
        roboteam_msgs::FieldLineSegment line;
        int sig;
        if (myPos.x < 0 && ROBOT_ID != KEEPER_ID) { // our side
            line = field.left_penalty_line;
            sig = -1;
        } else { // their side
            line = field.right_penalty_line;
            sig = 1;
        }
        double px1 = line.begin.x;
        double px2 = sig * (field.field_length/2 + 1); // extra margin should make sure this also works if the bot is beyond the goal line
        double py1 = line.begin.y;
        double py2 = line.end.y;
        if (py1 > py2) { // make sure py1 is always smaller than py2
            py1 = py2;
            py2 = line.begin.y;
        }
        // Define defense area sections
        Section bottomSec(px1, py1, px2, py1);
        Section topSec(px1, py2, px2, py2);
        Section verticalSec(px1, py1, px1, py2);
        // Find intersections 
        Vector2 intersect1 = posErrorSec.intersection(bottomSec);
        Vector2 intersect2 = posErrorSec.intersection(topSec);
        Vector2 intersect3 = posErrorSec.intersection(verticalSec);
        // Add safety margins to defense area points, which can now function as intermediate target points
        px1 -= sig * safetyMarginGoalAreas;
        py1 -= safetyMarginGoalAreas;
        py2 += safetyMarginGoalAreas;

        // If my posError section intersects the defense area (excluding margins) -> use intermediate target point
        if ((posErrorSec.pointOnLine(intersect1) && bottomSec.pointOnLine(intersect1))
            || (posErrorSec.pointOnLine(intersect2) && topSec.pointOnLine(intersect2))
            || (posErrorSec.pointOnLine(intersect3) && verticalSec.pointOnLine(intersect3))) {
            Vector2 interTarget = newTargetPos;
            double extra = 0.3;
            if (fabs(myPos.x) > fabs(px1)) {
                if(myPos.y < 0) {
                    interTarget = Vector2(px1 - sig * extra, py1);
                } else {
                    interTarget = Vector2(px1 - sig * extra, py2);
                }
            } else {
                if(myPos.y < newTargetPos.y) {
                    interTarget = Vector2(px1, py2 + extra);
                } else {
                    interTarget = Vector2(px1, py1 - extra);
                }
            }
            newTargetPos = interTarget;
        }
    }
    //////////////////////////////////
    return newTargetPos;
}


boost::optional<roboteam_msgs::RobotCommand> GoToPos::getVelCommand() {

    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    KEEPER_ID = blackboard->GetInt("KEEPER_ID");
    Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    

    if (HasDouble("pGainPosition")) {
        controller.setControlParam("pGainPosition", GetDouble("pGainPosition"));
    } else if (HasBool("pGainLarger") && GetBool("pGainLarger") && !grsim) {
        controller.setControlParam("pGainPosition", 6.5);
        controller.setControlParam("dGainPosition", 1.3);
    }
    if (HasDouble("iGainPosition")) {
        controller.setControlParam("iGainPosition", GetDouble("iGainPosition"));
    }
    if (HasDouble("dGainPosition")) {
        controller.setControlParam("dGainPosition", GetDouble("dGainPosition"));
    }
    if (HasDouble("pGainRotation")) {
        controller.setControlParam("pGainRotation", GetDouble("pGainRotation"));
    }
    if (HasDouble("iGainRotation")) {
        controller.setControlParam("iGainRotation", GetDouble("iGainRotation"));
    }
    if (HasDouble("dGainRotation")) {
        controller.setControlParam("dGainRotation", GetDouble("dGainRotation"));
    }
    if (HasDouble("maxSpeed")) {
        controller.setControlParam("maxSpeed", GetDouble("maxSpeed"));
    } else if (GetBool("lowSpeed")) {
        if (grsim) {
            controller.setControlParam("maxSpeed", 0.5);
        } else {
            controller.setControlParam("maxSpeed", 1.3);
        }
    }
    if (HasDouble("maxAngularVel")) {
        controller.setControlParam("maxAngularVel", GetDouble("maxAngularVel"));
    }
    if (HasDouble("thresholdTarget")) {
        controller.setControlParam("thresholdTarget", GetDouble("thresholdTarget"));
    }
    if (HasDouble("minTarget")) {
        controller.setControlParam("minTarget", GetDouble("minTarget"));
    }


    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();

    // Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(ROBOT_ID, true, world);
    roboteam_msgs::WorldRobot me;
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN_STREAM_NAMED(ROS_LOG_NAME, "Robot with this ID not found, ID: " << ROBOT_ID);
        failure = true;
        succeeded = false;
        return boost::none;
    }

    // Store some variables for easy access
    Vector2 myPos(me.pos);
    Vector2 myVel(me.vel);

    //TODO: temporary hack for use in rtt_jelle/BallPlacementAlt
    if (GetBool("driveBackward")) {
        targetPos = backwardPos;
    }

    // Determine angle goal and error
    double angleGoal;
    if (HasDouble("angleGoal")) {
        angleGoal = cleanAngle(GetDouble("angleGoal"));
    } else {
        angleGoal = me.angle;
    }
    
    double myAngle = me.angle;
    double angleError = cleanAngle(angleGoal - myAngle);
    double myAngularVel = me.w;

    // Determine how close we should get to the targetPos before we succeed
    double successDist;
    if (HasDouble("successDist")) {
        successDist = GetDouble("successDist");
    } else {
        successDist = 0.05;
    }

    // Check the input position
    if (targetPos == prevTargetPos) {
        targetPos = prevTargetPos;
    } else {
        targetPos = checkTargetPos(targetPos, myPos);
        prevTargetPos = targetPos;
    }

    if (HasBool("stayAwayFromBall") && GetBool("stayAwayFromBall")) {
        // ROS_INFO_STREAM("robot" << ROBOT_ID << " in stayAwayFromBall" );
        Vector2 ballPos(world.ball.pos);
        if ((ballPos - targetPos).length() < 0.7) {
            Vector2 diffVecNorm = (targetPos - ballPos).normalize();
            targetPos = ballPos + diffVecNorm.scale(0.7);
        }
    }

    Vector2 posError = targetPos - myPos;
    // If we are close enough to our target position and target orientation, then stop the robot and return success
    if (posError.length() < successDist && fabs(angleError) < 0.01) {
        successCounter++;
        if (successCounter >= 3) {
            succeeded = true;
            failure = false;
            return boost::none;
        }
    } else {
        successCounter = 0;
    }

    // // Limit the targetpos derivative (to apply a smoother command to the robot, which it can handle better)
    // static time_point prevTime = now();
    // int timeDiff = time_difference_milliseconds(prevTime, now()).count();
    // prevTime = now();
    // double max_diff = 2.0*timeDiff*0.001;
    // static Vector2 prevTarget = myPos + Vector2(0.001,0);
    // Vector2 smoothTargetPos = targetPos;
    // Vector2 targetDiff = targetPos - prevTarget;
    // if (targetDiff.length() > max_diff) {
    //     smoothTargetPos = (prevTarget + targetDiff.stretchToLength(max_diff));
    // }
    // prevTarget = smoothTargetPos;

    // drawer.setColor(255, 50, 50);
    // drawer.drawPoint("smoothTargetPos", smoothTargetPos);

    // Turn towards goal when error is too large
    static double posErrorRotationThreshold = 0.30;
    if (posError.length() > posErrorRotationThreshold && !GetBool("dontRotate")) {
        angleGoal = posError.angle();
        if (fabs(cleanAngle(angleGoal - myAngle)) > M_PI/2) {
            angleGoal = cleanAngle(angleGoal + M_PI); // chooses either driving forward or backwards
        }
        angleError = cleanAngle(angleGoal - myAngle);
        posErrorRotationThreshold = 0.29;
    } else {
        posErrorRotationThreshold = 0.30;
    }

    // Draw the line towards the target position
    drawer.setColor(0, 100, 100);
    drawer.drawLine("posError_" + std::to_string(ROBOT_ID), myPos, posError);
    drawer.setColor(0, 0, 0);

    // A vector to combine all the influences of different controllers (normal position controller, obstacle avoidance, defense area avoidance...)
    Vector2 sumOfForces(0.0, 0.0);

    // Position controller to steer the robot towards the target position
    sumOfForces = sumOfForces + controller.positionController(myPos, targetPos, myVel);

    // Robot avoidance
    if (HasBool("avoidRobots") && !GetBool("avoidRobots")) {
        // AvoidRobots defaults to true if not set
    } else {
        sumOfForces = avoidRobots(myPos, myVel, targetPos, sumOfForces);
    }

    // Ball avoidance
    if (HasBool("avoidBall") && GetBool("avoidBall")) {
        Vector2 ballPos = Vector2(world.ball.pos);
        sumOfForces = sumOfForces + avoidBall(ballPos, myPos, sumOfForces, targetPos, myVel);
    }
    // Defense area avoidance
    if (!(HasBool("enterDefenseAreas") && GetBool("enterDefenseAreas"))) {
        sumOfForces = avoidDefenseAreas(myPos, myVel, targetPos, sumOfForces);
    }


    // Limit the command derivative (to apply a smoother command to the robot, which it can handle better)
    static time_point prevTime = now();
    int timeDiff = time_difference_milliseconds(prevTime, now()).count();
    prevTime = now();
    double max_diff = 10.0*timeDiff*0.001;
    static Vector2 prevCommand;
    Vector2 commandDiff = sumOfForces - prevCommand;
    if (commandDiff.length() > max_diff) {
        sumOfForces = (prevCommand + commandDiff.stretchToLength(max_diff));
    }
    prevCommand = sumOfForces;
    //////////

    // Different velocity commands for grsim and real robot
    roboteam_msgs::RobotCommand command;
    Vector2 velCommand;
    if (grsim) {

        velCommand = worldToRobotFrame(sumOfForces, myAngle);   // Rotate the commands from world frame to robot frame
        double angularVelCommand = controller.rotationController(myAngle, angleGoal, posError, myAngularVel); // Rotation controller
        velCommand = controller.limitVel2(velCommand, angleError);          // Limit linear velocity
        angularVelCommand = controller.limitAngularVel(angularVelCommand);  // Limit angular velocity
        
        if ( HasBool("dontRotate") && GetBool("dontRotate") ) {
            command.w = 0;
        } else {
            command.w = angularVelCommand;
        }
    } else {
        velCommand = sumOfForces; //worldToRobotFrame(sumOfForces, myAngle);   // Rotate the commands from world frame to robot frame
        velCommand = controller.limitVel2(velCommand, angleError);
        // // The rotation of linear velocity to robot frame happens on the robot itself now
        // // Also, the robot has its own rotation controller now. Make sure this is enabled on the robot
        if ( (HasBool("dontRotate") && GetBool("dontRotate"))) {
                command.use_angle = false;
        } else {
                command.use_angle = true;
        }
        
        double angleCommand = angleGoal*16; // make sure it fits in the angularvel package
        command.w = angleCommand;
    }

    // Apply any max velocity that is set
    double maxVel = GetDouble("maxVelocity", 299792458.0);
    if (velCommand.length() > maxVel) {
    	velCommand = velCommand.stretchToLength(maxVel);
    }

    // Draw the target velocity vector in rqt-view (in red, oooh)
    drawer.setColor(255, 0, 0);
    drawer.drawLine("velTarget" + std::to_string(ROBOT_ID), myPos, velCommand.scale(0.5));
    drawer.setColor(0, 0, 0);

    // fill the rest of command message
    command.id = ROBOT_ID;
    command.dribbler = GetBool("dribbler");
    command.x_vel = velCommand.x;
    command.y_vel = velCommand.y;

    return command;
}


bt::Node::Status GoToPos::Update() {

    boost::optional<roboteam_msgs::RobotCommand> command = getVelCommand();
    if (command) {
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        if (command->x_vel != command->x_vel) {
        	ROS_ERROR("Tried to send NAN");
        	return Status::Invalid;
        }
        pub.publish(*command);
        return Status::Running;
    } else {
        sendStopCommand(ROBOT_ID);

        if (succeeded) {
            return Status::Success;
        } else if (failure) {
            return Status::Failure;
        } else {
            return Status::Invalid;
        }
    }
}

} // rtt
