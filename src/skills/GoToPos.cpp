#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ros/ros.h>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/conditions/IsBallInDefenseArea.h"

#define RTT_CURRENT_DEBUG_TAG GoToPos

namespace rtt {

RTT_REGISTER_SKILL(GoToPos);

GoToPos::GoToPos(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)

        {
            ROS_DEBUG_STREAM_NAMED("GoToPos", "New GoToPos started for robot " << blackboard->GetInt("ROBOT_ID"));
            ROS_DEBUG_STREAM_NAMED("GoToPos", "Blackboard : " << blackboard->toString());

            succeeded = false;
            failure = false;
            controller.Initialize(blackboard->GetInt("ROBOT_ID"));

            // Debug info
            ros::NodeHandle n;
            myPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myPosTopic", 1000);
            myVelTopic = n.advertise<roboteam_msgs::WorldRobot>("myVelTopic", 1000);
            myTargetPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myTargetPosTopic", 1000);

            //DEFAULTS
            ros::param::getCached("robot_output_target", robot_output_target);
            if (robot_output_target == "grsim") {
                safetyMarginGoalAreas = 0.1;
                marginOutsideField = 0.3;
                avoidRobotsGain = 0.010;
                cushionGain = 0.06;
                minDist = 0.01; // avoidance force does not increase further when dist becomes smaller that minDist
                maxDist = 0.3; // no force is exerted when dist is larger than maxDist
            } else if (robot_output_target == "serial") {
                safetyMarginGoalAreas = 0.1;
                marginOutsideField = -0.1; //ALTERED CURRENTLY FOR THE DEMOFIELD, NORMALLY: 0.3
                avoidRobotsGain = 0.005;
                cushionGain = 0.06;
                minDist = 0.01; // avoidance force does not increase further when dist becomes smaller that minDist
                maxDist = 0.3; // no force is exerted when dist is larger than maxDist
            }
            
            //PROCESS BLACKBOARD
            if (HasDouble("avoidRobotsGain")) {
                avoidRobotsGain = GetDouble("avoidRobotsGain");
            }
            if (HasDouble("cushionGain")) {
                cushionGain = GetDouble("cushionGain");
            }
            if (HasDouble("minDist")) {
                minDist = GetDouble("minDist");
            }
            if (HasDouble("maxDist")) {
                maxDist = GetDouble("maxDist");
            }
        }


void GoToPos::sendStopCommand(uint id) {
    roboteam_msgs::RobotCommand command = controller.getStopCommand(id);

    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}


// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them7
// Based on this algorithm: https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
// Tweaked by Jelle, to prevent division by zero and having a maximum force that can be applied (by using a minimum distance)
Vector2 GoToPos::getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna) {

    Vector2 ahead = myPos + antenna; // tip of the antenna
    Vector2 closestPoint = antenna.closestPointOnVector(myPos, otherRobotPos);
    Vector2 force = closestPoint - otherRobotPos; // Vector from other robot to closest point on antenna.
    double distToAntenna = force.length(); // distance between closest point on antenna and other robot.
    double distToMe = (myPos - otherRobotPos).length(); // distance between me and other robot, used for scaling the force.

    // Check if the point lies within the antenna, so not on the edges. 
    // Then calculate the needed avoidance force.
    if ((closestPoint - myPos).length() > 0.001 && (ahead - closestPoint).length() > 0.001 && distToAntenna <= maxDist){
        if(distToAntenna > minDist) {
            force = force.stretchToLength(avoidRobotsGain/distToAntenna/distToMe);
        } else if(distToAntenna > 0.0005) { // avoid division by 0
            force = force.stretchToLength(avoidRobotsGain/minDist/distToMe);
        } else { // distToAntenna almost zero -> force direction becomes 
            force = (antenna.rotate(M_PI/2)).stretchToLength(avoidRobotsGain/minDist/distToMe);
        }
    } else {
        force = Vector2(0,0);
    }
    return force;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid crashing into other robots
Vector2 GoToPos::avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos) {
    roboteam_msgs::World world = LastWorld::get();
    Vector2 posError = targetPos - myPos;
    // The antenna is a vector starting at the robot position in the direction in which it wants to go (scaled to the robot speed)
    double minAntenna = 0.5; // antenna has a minimal length, to make sure it still avoids when it starts from a standstill.
    Vector2 antenna = posError.stretchToLength(minAntenna + myVel.length()*0.7);
    // The antenna will not grow beyond its minimum value when moving backwards
    if (myVel.dot(antenna)<0 ){
        antenna = posError.stretchToLength(minAntenna);
    }
    // The antenna is never larger than the position error
    if (posError.length() < antenna.length()) {
            antenna = posError;
    }

    // For all robots in the field that are closer than the antenna length to our robot: ..
    // ..determine if they exert a repelling force and add all these forces.
    // A 'cushion force' that damps out the 'crash velocity' was added by Jelle to prevent hard collisions that cant be avoided.
    // To find a better way of taking velocities into account when avoiding robots, Jelle replaced the use of estimated future positions ..
    // ..by the concept of a 'relative antenna', which is a rotated version of the antenna for each robot that is close, ..
    // ..based on the relative velocity between me and the other robot.
    Vector2 sumOfForces;
    Vector2 sumOfCushions;
    for (auto const currentRobot : world.us) {
        if (currentRobot.id != ROBOT_ID) {
            Vector2 otherRobotPos(currentRobot.pos);
            Vector2 otherRobotVel(currentRobot.vel);
            double distToRobot = (otherRobotPos - myPos).length();

            Vector2 relativeVel = myVel - otherRobotVel;
            Vector2 relativeAntenna = antenna;
            if (relativeVel.length()>0.2 && myVel.length()>0.1) {
                relativeAntenna = antenna.rotate(relativeVel.angle()-myVel.angle());
            }
            
            if (distToRobot <= antenna.length() && relativeAntenna.dot(myVel) > 0) {
                Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, relativeAntenna);
                drawer.setColor(0, 0, 255);
                drawer.drawLine("avoidForce" + std::to_string(currentRobot.id),otherRobotPos,forceVector.scale(0.5));
                sumOfForces = sumOfForces + forceVector; //We add the avoidance forceVector to the total

                // Determine the crash velocity (velocity at which the robots would crash into eachother)..
                // ..and determine the 'cushion force' to damp out this crash velocity.
                Vector2 crashVel = (otherRobotVel - myVel).project2(myPos - otherRobotPos);
                Vector2 cushionForce = crashVel.scale(cushionGain / distToRobot);
                //I will not respond to a negative crashVel or a crashVel not caused by me
                if (crashVel.dot(myPos - otherRobotPos) < 0 || myVel.dot(otherRobotPos - myPos) < 0.01) {
                    cushionForce = Vector2(0,0);
                }
                sumOfCushions = sumOfCushions + cushionForce; // We add the cushion force to the total
                drawer.setColor(255, 0, 255);
                drawer.drawLine("cushionForce" + std::to_string(currentRobot.id),otherRobotPos,cushionForce.scale(0.5));
            }
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
            Vector2 otherRobotPos(world.them.at(i).pos);
            Vector2 otherRobotVel(world.them.at(i).vel);
            double distToRobot = (otherRobotPos - myPos).length();

            Vector2 relativeVel = myVel - otherRobotVel;
            Vector2 relativeAntenna = antenna;
            if (relativeVel.length()>0.2 && myVel.length()>0.1) {
                relativeAntenna = antenna.rotate(relativeVel.angle()-myVel.angle());
            }
            
            if (distToRobot <= antenna.length() && relativeAntenna.dot(myVel) > 0) {
                Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, relativeAntenna);
                drawer.setColor(0, 0, 255);
                drawer.drawLine("avoidForce" + std::to_string(i),otherRobotPos,forceVector.scale(0.5));
                sumOfForces = sumOfForces + forceVector; //We add the avoidance forceVector to the total
                //'crashVel' is the velocity at which the robots would crash into eachother
                Vector2 crashVel = (otherRobotVel - myVel).project2(myPos - otherRobotPos);
                // A 'cushionForce' is calculated to decrease the crashVel. Size depends on crash velocity and proximity.
                Vector2 cushionForce = crashVel.scale(cushionGain / distToRobot);    //////////////////TWEAK///////////////////////
                //I will not respond to a negative crashVel or a crashVel not caused by me
                if (crashVel.dot(myPos - otherRobotPos) < 0 || myVel.dot(otherRobotPos - myPos) < 0.01) {
                    cushionForce = Vector2(0,0);
                }
                sumOfCushions = sumOfCushions + cushionForce; //We add the cushion force to the total
                drawer.setColor(255, 0, 255);
                drawer.drawLine("cushionForce" + std::to_string(i),otherRobotPos,cushionForce.scale(0.5));
            }
    }

    //draw cusionForce in rqt
    // drawer.setColor(255, 0, 255);
    // drawer.drawLine("cushionForce", myPos, sumOfCushions.scale(0.5));
    // //draw sumOfForces
    // drawer.setColor(0, 0, 255);
    // drawer.drawLine("sumOfForces", myPos, sumOfForces.scale(0.5));

    drawer.setColor(0, 155, 155);
    drawer.drawPoint("antenna" + std::to_string(ROBOT_ID),myPos+antenna);

    sumOfForces = sumOfForces + sumOfCushions;
    return sumOfForces;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid the goal areas by moving only parralel to the goal area when close
Vector2 GoToPos::avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces) {
    Vector2 posError = targetPos - myPos;

    if (ROBOT_ID != KEEPER_ID) {
        Vector2 distToOurDefenseArea = getDistToDefenseArea(true, myPos, safetyMarginGoalAreas);
        if (isWithinDefenseArea(true, myPos, safetyMarginGoalAreas)) {
            // If we are already in the defense area, it's best just to drive straight out of it and not to add any weird forces
        } else if ((distToOurDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToOurDefenseArea) > 0) {
            if (sumOfForces.dot(distToOurDefenseArea.rotate(0.5*M_PI)) > 0) {
                sumOfForces = distToOurDefenseArea.rotate(0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
            } else {
                sumOfForces = distToOurDefenseArea.rotate(-0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
            }
        }
    }

    Vector2 distToTheirDefenseArea = getDistToDefenseArea(false, myPos, safetyMarginGoalAreas);
    if (isWithinDefenseArea(false, myPos, safetyMarginGoalAreas)) {
        // If we are already in the defense area, it's best just to drive straight out of it and not to add any weird forces
    } else if ((distToTheirDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToTheirDefenseArea) > 0) {
        if (sumOfForces.dot(distToTheirDefenseArea.rotate(0.5*M_PI)) > 0) {
            sumOfForces = distToTheirDefenseArea.rotate(0.5*M_PI).scale(sumOfForces.length() / distToTheirDefenseArea.length());
        } else {
            sumOfForces = distToTheirDefenseArea.rotate(-0.5*M_PI).scale(sumOfForces.length() / distToTheirDefenseArea.length());
        }
    }

    return sumOfForces;
}


Vector2 GoToPos::avoidBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces, Vector2 targetPos, Vector2 myVel) {
    // Vector2 diff = ballPos - myPos;
    // double theta = fabs(cleanAngle(diff.angle() - sumOfForces.angle()));

    // if (theta < (0.5 * M_PI)) {
    //     if (fabs(theta) < .00001) theta = 0.01;
    //     // double force = theta / (0.5 * M_PI);
    //     Vector2 projectedBall = ballPos.project(myPos, myPos + sumOfForces);
    //     Vector2 ballForce = projectedBall - ballPos;
    //     sumOfForces = sumOfForces + ballForce * 5;
    // }

    // roboteam_msgs::World world = LastWorld::get();
    // Vector2 ballVel(world.ball.)

    // The antenna is a vector starting at the robot position in the direction in which it is driving (scaled to the robot speed)
    double lookingDistance = 1.0;
    Vector2 posError = targetPos - myPos;

    Vector2 antenna = Vector2(lookingDistance, 0.0).rotate(posError.angle());
    antenna = antenna.scale(myVel.length()*1.0); // magic scaling constant

    sumOfForces = sumOfForces + getForceVectorFromRobot(myPos, ballPos, antenna);

    return sumOfForces;
}


// Makes sure that the given target position is not inside a goal area, or (too far) outside the field. "Too far" is specified by the class variable marginOutsideField
Vector2 GoToPos::checkTargetPos(Vector2 targetPos) {
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

    double xGoal = targetPos.x;
    double yGoal = targetPos.y;

    double newMargin = marginOutsideField;
    // In and around the goal the robot cannot cross the backline.
    if (fabs(yGoal) < (field.goal_width/2 + 0.2)) {
        newMargin = -0.1;
    }
    if (fabs(yGoal) < (field.goal_width/2)) {
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
    if (ROBOT_ID != KEEPER_ID && !(HasBool("enterDefenseAreas") && GetBool("enterDefenseAreas"))) {
        // If the target position is in our defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea(true, newTargetPos, safetyMarginGoalAreas)) {
            Vector2 distToOurDefenseArea = getDistToDefenseArea(true, newTargetPos, safetyMarginGoalAreas);
            newTargetPos = newTargetPos + distToOurDefenseArea;
        }

        // If the target position is in their defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea(false, newTargetPos, safetyMarginGoalAreas)) {
            Vector2 distToTheirDefenseArea = getDistToDefenseArea(false, newTargetPos, safetyMarginGoalAreas);
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


    

    return newTargetPos;
}


boost::optional<roboteam_msgs::RobotCommand> GoToPos::getVelCommand() {

    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    KEEPER_ID = blackboard->GetInt("KEEPER_ID");

    if (HasBool("pGainLarger") && GetBool("pGainLarger") && robot_output_target == "serial") {
        controller.setControlParam("pGainPosition", 3.5);
    } else {
        controller.setPresetControlParams();
    }

    if (HasDouble("pGainPosition")) {
        controller.setControlParam("pGainPosition", GetDouble("pGainPosition"));
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
    }
    if (HasDouble("maxAngularVel")) {
        controller.setControlParam("maxAngularVel", GetDouble("maxAngularVel"));
    }



    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();

    // Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(ROBOT_ID);
    roboteam_msgs::WorldRobot me;
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN_STREAM("GoToPos: robot with this ID not found, ID: " << ROBOT_ID);
        failure = true;
        succeeded = false;
        return boost::none;
    }


    // Check the input position
    if (targetPos == prevTargetPos) {
        targetPos = prevTargetPos;
    } else {
        targetPos = checkTargetPos(targetPos);
        prevTargetPos = targetPos;
    }

    if (HasBool("stayAwayFromBall") && GetBool("stayAwayFromBall")) {
        // ROS_INFO_STREAM("robot" << ROBOT_ID << " in stayAwayFromBall" );
        roboteam_msgs::World world = LastWorld::get();
        Vector2 ballPos(world.ball.pos);
        if ((ballPos - targetPos).length() < 0.7) {
            Vector2 diffVecNorm = (targetPos - ballPos).normalize();
            targetPos = ballPos + diffVecNorm.scale(0.7);
        }
    }

    // Store some variables for easy access
    Vector2 myPos(me.pos);
    Vector2 myVel(me.vel);
    Vector2 posError = targetPos - myPos;

    // For the 1v1 demo: if the robot (which is controlled by a human) gets inside our defense area, ..
    // ..generate a targetpos just outside the defense area based on the robot position
    if (HasBool("goalOutsideDefenseArea") && GetBool("goalOutsideDefenseArea")) {
        targetPos = checkTargetPos(myPos);
    }

    // Draw the line towards the target position
    drawer.setColor(0, 100, 100);
    drawer.drawLine("posError_" + std::to_string(ROBOT_ID), myPos, posError);
    drawer.setColor(0, 0, 0);

    double angleGoal;
    if (HasDouble("angleGoal")) {
        angleGoal = cleanAngle(GetDouble("angleGoal"));
    } else {
        angleGoal = me.angle;
    }

    if (posError.length() > 0.5) {
        angleGoal = posError.angle();
    } 

    double myAngle = me.angle;
    double angleError = cleanAngle(angleGoal - myAngle);
    double myAngularVel = me.w;

    // Determine how close we should get to the targetPos before we succeed
    double successDist;
    if (HasDouble("successDist")) {
        successDist = GetDouble("successDist");
    } else {
        successDist = 0.03;
    }

    // If we are close enough to our target position and target orientation, then stop the robot and return success
    if (posError.length() < successDist && fabs(angleError) < 0.08) {
        successCounter++;
        if (successCounter >= 3) {
            //sendStopCommand(ROBOT_ID);/////////////////////////////////////////////////////////////////////////////////
            succeeded = true;
            failure = false;
            return boost::none;
        }
    } else {
        successCounter = 0;
    }

    // A vector to combine all the influences of different controllers (normal position controller, obstacle avoidance, defense area avoidance...)
    Vector2 sumOfForces(0.0, 0.0);

    // Position controller to steer the robot towards the target position
    sumOfForces = sumOfForces + controller.positionController(myPos, targetPos, myVel);

    // Robot avoidance
    if (HasBool("avoidRobots") && !GetBool("avoidRobots")) {
        // AvoidRobots defaults to true if not set
    } else {
        Vector2 newSumOfForces = sumOfForces + avoidRobots(myPos, myVel, targetPos);
        if (newSumOfForces.length() >= 1.5) {
            angleGoal = newSumOfForces.angle();
            angleError = cleanAngle(angleGoal - myAngle);
        }
        sumOfForces = newSumOfForces;
    }

    // Rotation controller to make sure the robot reaches its angleGoal
    double angularVelTarget = controller.rotationController(myAngle, angleGoal, posError, myAngularVel);

    

    // Defense area avoidance
    if (!(HasBool("enterDefenseAreas") && GetBool("enterDefenseAreas"))) {
        sumOfForces = avoidDefenseAreas(myPos, myVel, targetPos, sumOfForces);
    }

    // Ball avoidance
    if (HasBool("avoidBall") && GetBool("avoidBall")) {
        Vector2 ballPos = Vector2(world.ball.pos);
        sumOfForces = avoidBall(ballPos, myPos, sumOfForces, targetPos, myVel);
    }

    // Draw the target velocity vector in rqt-view (in red, oooh)
    drawer.setColor(255, 0, 0);
    drawer.drawLine("velTarget" + std::to_string(ROBOT_ID), myPos, sumOfForces.scale(0.5));
    drawer.setColor(0, 0, 0);

    // TEMPORARILY DRAW BALL VEL (FOR DEBUGGING)
    // Vector2 ballPos = Vector2(world.ball.pos);
    // Vector2 ballVel = Vector2(world.ball.vel);
    // drawer.setColor(255, 0, 0);
    // drawer.drawLine("balVel", ballPos, ballVel);

    // Rotate the commands from world frame to robot frame
    Vector2 velTarget = worldToRobotFrame(sumOfForces, myAngle);

    // BEUN Solution for now: strafing needs more gain
    if (HasDouble("strafeGain")) {
        velTarget.y = velTarget.y * GetDouble("strafeGain");
    }
    
    // Velocity controller
    // Vector2 velCommand = controller.velocityController(myVelRobotFrame, velTarget);
    
    Vector2 velCommand = velTarget;

    // Limit angular and linear velocity
    velCommand = controller.limitVel(velCommand, angularVelTarget);
    angularVelTarget = controller.limitAngularVel(angularVelTarget);

    double maxVel = GetDouble("maxVelocity", 299792458.0);
    if (velCommand.length() > maxVel) {
    	velCommand = velCommand.stretchToLength(maxVel);
    }

    if (posError.length() >= 1.0 && fabs(angleError) >= 1.0) {
        if (velCommand.length() >= 1.5) {
            velCommand = velCommand.scale(1.5 / velCommand.length());
        }
    }


    // Fill the command message
    roboteam_msgs::RobotCommand command;
    command.id = ROBOT_ID;
    command.x_vel = velCommand.x;
    command.y_vel = velCommand.y;
    if ( !(HasBool("dontRotate") && GetBool("dontRotate")) ) {
        command.w = angularVelTarget;
    }
    command.dribbler = GetBool("dribbler");

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
