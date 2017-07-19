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
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"


#define RTT_CURRENT_DEBUG_TAG GoToPos

namespace rtt {

RTT_REGISTER_SKILL(GoToPos);

GoToPos::GoToPos(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)

        {
            succeeded = false;
            failure = false;
            ros::NodeHandle n;
            myPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myPosTopic", 1000);
            myVelTopic = n.advertise<roboteam_msgs::WorldRobot>("myVelTopic", 1000);
            myTargetPosTopic = n.advertise<roboteam_msgs::WorldRobot>("myTargetPosTopic", 1000);
            controller.Initialize(blackboard->GetInt("ROBOT_ID"));

            safetyMarginGoalAreas = 0.2;
            marginOutsideField = 0.3;
            avoidRobotsGain = 0.2;
        }


void GoToPos::sendStopCommand(uint id) {
    roboteam_msgs::RobotCommand command = controller.getStopCommand(id);

    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}


// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them7
// Based on this algorithm: https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
Vector2 GoToPos::getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna, Vector2 targetPos) {

    Vector2 forceVector;

    Vector2 ahead = myPos + antenna;
    Vector2 closestPoint = antenna.closestPointOnVector(myPos, otherRobotPos);

    double dist = (closestPoint - otherRobotPos).length();
    if (closestPoint != myPos && closestPoint != ahead && dist <= 0.3) {
        Vector2 force = closestPoint - otherRobotPos;
        force = force.scale(avoidRobotsGain / (force.length() * force.length()) );
        force = force.scale(1.0 / (closestPoint - myPos).length());
        forceVector = force;
    }

    return forceVector;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid crashing into other robots
Vector2 GoToPos::avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos) {
    roboteam_msgs::World world = LastWorld::get();

    Vector2 posError = targetPos - myPos;
    double lookingDistance = 1.0; // default
    if (lookingDistance > (posError.length())) {
        lookingDistance = posError.length();
    }

    // The antenna is a vector starting at the robot position in the direction in which it is driving (scaled to the robot speed)
    Vector2 antenna = Vector2(lookingDistance, 0.0).rotate(posError.angle());
    antenna = antenna.scale(myVel.length()*1.0); // magic scaling constant

    // Draw the antenna in rqt-view
    drawer.setColor(255, 0, 0);
    drawer.drawLine("antenna", myPos, antenna);

    // For all robots in the field that are closer than the lookingDistance to our robot, determine if they exert a repelling force and add all these forces
    Vector2 sumOfForces;
    for (auto const currentRobot : world.us) {
        if (currentRobot.id != ROBOT_ID) {
            Vector2 otherRobotPos(currentRobot.pos);
            Vector2 otherRobotVel(currentRobot.vel);
            double distToRobot = (otherRobotPos - myPos).length();
            Vector2 otherRobotFuturePos = otherRobotPos + otherRobotVel.scale(distToRobot / myVel.length());
            if ((otherRobotFuturePos - myPos).length() <= lookingDistance) {
                Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotFuturePos, antenna, targetPos);
                sumOfForces = sumOfForces + forceVector;
            }
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        Vector2 otherRobotPos(world.them.at(i).pos);
        Vector2 otherRobotVel(world.them.at(i).vel);
        double distToRobot = (otherRobotPos - myPos).length();
        Vector2 otherRobotFuturePos = otherRobotPos + otherRobotVel.scale(distToRobot / myVel.length());
        if ((otherRobotFuturePos - myPos).length() <= lookingDistance) {
            Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotFuturePos, antenna, targetPos);
            sumOfForces = sumOfForces + forceVector;
        }
    }

    drawer.setColor(0, 255, 255);
    drawer.drawLine("sumOfForces", myPos, sumOfForces);
    drawer.setColor(0, 0, 0);

    return sumOfForces;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid the goal areas by moving only parralel to the goal area when close
Vector2 GoToPos::avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces) {
    Vector2 posError = targetPos - myPos;

    if (ROBOT_ID != KEEPER_ID) {
        Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", myPos, 0.0);
        if (fabs(distToOurDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToOurDefenseArea) > 0) {
            if (sumOfForces.dot(distToOurDefenseArea.rotate(0.5*M_PI)) > 0) {
                sumOfForces = distToOurDefenseArea.rotate(0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
            } else {
                sumOfForces = distToOurDefenseArea.rotate(-0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
            }
        }
    }

    Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", myPos, 0.0);
    if (fabs(distToTheirDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToTheirDefenseArea) > 0) {
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

    sumOfForces = sumOfForces + getForceVectorFromRobot(myPos, ballPos, antenna, targetPos);

    return sumOfForces;
}


// Makes sure that the given target position is not inside a goal area, or (too far) outside the field. "Too far" is specified by the class variable marginOutsideField
Vector2 GoToPos::checkTargetPos(Vector2 targetPos) {
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

    double xGoal = targetPos.x;
    double yGoal = targetPos.y;

    // We should not go outside the field close to the goal areas
    if (fabs(yGoal) < (field.goal_width/2 + safetyMarginGoalAreas)) {
        marginOutsideField = 0.0;
    }

    // If the target position is outside of the field + margins, then change the target position to the closest point within this margin
    // if (GetBool("ignoreFieldBounds", false)) {
        if (xGoal > (field.field_length/2+marginOutsideField) || xGoal < (-field.field_length/2-marginOutsideField)) {
            xGoal = signum(xGoal)*(field.field_length/2+marginOutsideField);
        }
        if (yGoal > (field.field_width/2+marginOutsideField) || yGoal < (-field.field_width/2-marginOutsideField)) {
            yGoal = signum(yGoal)*(field.field_width/2+marginOutsideField);
        }
    // }

    Vector2 newTargetPos(xGoal, yGoal);

    // If the current robot is not a keeper, we should take into account that it cannot enter the defense area
    if (ROBOT_ID != KEEPER_ID) {
    // if (ROBOT_ID != KEEPER_ID && !(HasBool("enterDefenseAreas") && GetBool("enterDefenseAreas"))) {
        // If the target position is in our defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea(true, newTargetPos)) {
            Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", newTargetPos, safetyMarginGoalAreas);
            newTargetPos = newTargetPos + distToOurDefenseArea;
        }

        // If the target position is in their defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea(false, newTargetPos)) {
            Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", newTargetPos, safetyMarginGoalAreas);
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


    if (HasBool("stayAwayFromBall") && GetBool("stayAwayFromBall")) {
        roboteam_msgs::World world = LastWorld::get();
        Vector2 ballPos(world.ball.pos);
        if ((ballPos - newTargetPos).length() < 0.7) {
            Vector2 diffVecNorm = (newTargetPos - ballPos).normalize();
            newTargetPos = ballPos + diffVecNorm.scale(0.7);
        }
    }

    return newTargetPos;
}


boost::optional<roboteam_msgs::RobotCommand> GoToPos::getVelCommand() {

    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    KEEPER_ID = blackboard->GetInt("KEEPER_ID");

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

    // drawer.drawPoint("targetPosOld_" + std::to_string(ROBOT_ID), targetPos);

    // Check the input position
    if (targetPos == prevTargetPos) {
        targetPos = prevTargetPos;
    } else {
        targetPos = checkTargetPos(targetPos);
        prevTargetPos = targetPos;
    }

    // Store some variables for easy access
    Vector2 myPos(me.pos);
    Vector2 myVel(me.vel);
    Vector2 posError = targetPos - myPos;

    // Draw the line towards the target position
    drawer.setColor(0, 100, 100);
    drawer.drawLine("posError_" + std::to_string(ROBOT_ID), myPos, posError);
    // drawer.drawPoint("targetPos_" + std::to_string(ROBOT_ID), targetPos);
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
            sendStopCommand(ROBOT_ID);
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

    // Rotation controller to make sure the robot reaches its angleGoal
    double angularVelTarget = controller.rotationController(myAngle, angleGoal, posError, myAngularVel);

    // Robot avoidance
    if (HasBool("avoidRobots") && GetBool("avoidRobots")) {
        sumOfForces = sumOfForces + avoidRobots(myPos, myVel, targetPos);
        // Possibly necessary to scale the velocity to the maxSpeed again after avoidRobots:
        // if (posError.length() > 0.5) {
        //     if (sumOfForces.length() < maxSpeed) {
        //         sumOfForces = sumOfForces.scale(maxSpeed / sumOfForces.length());
        //     }
        // }
    }

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
    // drawer.setColor(255, 0, 0);
    // drawer.drawLine("velTarget" + std::to_string(ROBOT_ID), myPos, sumOfForces);
    // drawer.setColor(0, 0, 0);

    // Rotate the commands from world frame to robot frame
    Vector2 velTarget = worldToRobotFrame(sumOfForces, myAngle);


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
    command.w = angularVelTarget;
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
