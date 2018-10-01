#pragma once

#include "ros/ros.h"

#include <vector>
#include <boost/optional.hpp>

#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/utils/utils.h"
#include "roboteam_tactics/utils/Control.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Draw.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_tactics/conditions/IsInDefenseArea.h"


namespace rtt{

//All the avoid functions neccesarry in GoToPos, since there actually still mathamathically relevant, so don't wanna mess them up during rewite of GoToPos

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
                // Jelle: Remember why did I did it like this, and not simply use relativeVel directly as direction of my antenna. Becomes untable?
                // Koen: Done like this to take any rotation action already in place into acount, if they are already rotating away,
                // like the opponnent already reacted, then we have to rotate less.
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
        }

        drawer.setColor(0, 155, 155);
        drawer.drawPoint("antenna" + std::to_string(ROBOT_ID),myPos+antenna);

        return sumOfForces;
    }


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid the goal areas by moving only parralel to the goal area when close
    Vector2 GoToPos::avoidDefenseAreas(Vector2 myPos, Vector2 targetPos, Vector2 sumOfForces) {
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
        if (fabs(yGoal) < (field.goal_width/2 + 1.0)) {
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


        if (!(HasBool("enterDefenseAreas") && GetBool("enterDefenseAreas"))) {
            // If the target position is in our defense area, then subtract the vector difference between the defense area and the target position
            if (ROBOT_ID != KEEPER_ID && isWithinDefenseArea(true, newTargetPos, safetyMarginGoalAreas)) {
                Vector2 distToOurDefenseArea = getDistToDefenseArea(true, newTargetPos, safetyMarginGoalAreas);
                drawer.setColor(50, 50, 50);
                drawer.drawLine("defAreaLine", newTargetPos, distToOurDefenseArea);
                newTargetPos = newTargetPos + distToOurDefenseArea;
            }

            // If the target position is in their defense area, then subtract the vector difference between the defense area and the target position
            if (isWithinDefenseArea(false, newTargetPos, safetyMarginGoalAreas)) {
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



} // rtt
