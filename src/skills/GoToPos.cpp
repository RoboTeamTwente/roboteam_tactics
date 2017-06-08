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

            safetyMarginGoalAreas = 0.0;
            marginOutsideField = -0.1;
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
    Vector2 posError = targetPos - myPos;

    Vector2 ahead = myPos + antenna;
    if ((ahead - otherRobotPos).length() < 0.2) {
        Vector2 force = ahead - otherRobotPos;
        force = force.scale(avoidRobotsGain / (force.length() * force.length()));
        force = limitAngleDiff(force, posError, 0.5*M_PI);
        forceVector = force;
    }

    Vector2 ahead2 = myPos + antenna.scale(0.5);
    if ((ahead2 - otherRobotPos).length() < 0.2) {
        Vector2 force = ahead2 - otherRobotPos;
        force = force.scale(avoidRobotsGain / (force.length() * force.length()));
        force = limitAngleDiff(force, posError, 0.5*M_PI);
        forceVector = forceVector + force;
    }

    if ((myPos - otherRobotPos).length() < 0.3) {
        Vector2 force = myPos - otherRobotPos;
        force = force.scale((avoidRobotsGain) / (force.length() * force.length()));
        force = limitAngleDiff(force, posError, 0.5*M_PI);
        forceVector = forceVector + force;
    }

    return forceVector;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid crashing into other robots
Vector2 GoToPos::avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos) {
    roboteam_msgs::World world = LastWorld::get();

    Vector2 posError = targetPos - myPos;
    double lookingDistance = 0.5; // default
    if (lookingDistance > (posError.length())) {
        lookingDistance = posError.length();
    }
    
    // The antenna is a vector starting at the robot position in the direction in which it is driving (scaled to the robot speed)
    Vector2 antenna = Vector2(lookingDistance, 0.0).rotate(posError.angle());
    antenna = antenna.scale(myVel.length()*1.5); // magic scaling constant

    // Draw the antenna in rqt-view
    // drawer.drawLine("antenna", myPos, antenna);

    // For all robots in the field that are closer than the lookingDistance to our robot, determine if they exert a repelling force and add all these forces
    Vector2 sumOfForces;
    for (auto const currentRobot : world.us) {
        if (currentRobot.id != ROBOT_ID) {
            Vector2 otherRobotPos(currentRobot.pos);
            if ((otherRobotPos - myPos).length() <= lookingDistance) {
                Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, antenna, targetPos);
                sumOfForces = sumOfForces + forceVector;
            }
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        Vector2 otherRobotPos(world.them.at(i).pos);
        if ((otherRobotPos - myPos).length() <= lookingDistance) {
            Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, antenna, targetPos);
            sumOfForces = sumOfForces + forceVector; 
        }
    }

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


Vector2 GoToPos::avoidBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces) {
    Vector2 diff = ballPos - myPos;
    double theta = fabs(cleanAngle(diff.angle() - sumOfForces.angle()));

    if (theta < (0.5 * M_PI)) {
        if (theta == 0) theta = 0.01;
        // double force = theta / (0.5 * M_PI);
        Vector2 projectedBall = ballPos.project(myPos, myPos + sumOfForces);
        Vector2 ballForce = projectedBall - ballPos;
        sumOfForces = sumOfForces + ballForce * 5;
    }

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
        // TODO: Doesn't work if normalize_field (rosparam in mini.launch) is true. Disabled until further notice    

        // // If the target position is in our defense area, then subtract the vector difference between the defense area and the target position
        // if (isWithinDefenseArea("our defense area", newTargetPos)) {
            // Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", newTargetPos, safetyMarginGoalAreas);
            // newTargetPos = newTargetPos + distToOurDefenseArea;
        // }

        // // If the target position is in their defense area, then subtract the vector difference between the defense area and the target position
        // if (isWithinDefenseArea("their defense area", newTargetPos)) {
            // Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", newTargetPos, safetyMarginGoalAreas);
            // newTargetPos = newTargetPos + distToTheirDefenseArea;
        // }
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
    KEEPER_ID = GetInt("KEEPER_ID", 100);

    if (HasDouble("maxSpeed")) {
        controller.setControlParam("maxSpeed", GetDouble("maxSpeed"));
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
        if (posError.length() > 0.5) {
            angleGoal = posError.angle();
        } else {
            angleGoal = me.angle;
        }
    }

    double myAngle = me.angle;
    double angleError = angleGoal - myAngle;

    // @DEBUG info: 
    myPosTopic.publish(me);
    roboteam_msgs::WorldRobot targetPosPub;
    targetPosPub.pos.x = targetPos.x;
    targetPosPub.pos.y = targetPos.y;
    targetPosPub.angle = angleGoal;
    myTargetPosTopic.publish(targetPosPub);

    // Determine how close we should get to the targetPos before we succeed
    double successDist;
    if (HasDouble("successDist")) {
        successDist = GetDouble("successDist");
    } else {
        successDist = 0.02;
    }

    // If we are close enough to our target position and target orientation, then stop the robot and return success
    if (posError.length() < successDist && fabs(angleError) < 0.1) {
        successCounter++;
        if (successCounter >= 4) {
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
    sumOfForces = sumOfForces + controller.positionController(myPos, targetPos);


    // Rotation controller to make sure the robot reaches its angleGoal
    double angularVelTarget = controller.rotationController(myAngle, angleGoal, posError);

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
    // if (HasBool("avoidDefenseAreas") && GetBool("avoidDefenseAreas")) {
    sumOfForces = avoidDefenseAreas(myPos, myVel, targetPos, sumOfForces);
    // }

    // Ball avoidance
    if (HasBool("avoidBall") && GetBool("avoidBall")) {
        Vector2 ballPos = Vector2(world.ball.pos);
        sumOfForces = avoidBall(ballPos, myPos, sumOfForces);
    }

    // Draw the target velocity vector in rqt-view (in red, oooh)
    // drawer.setColor(255, 0, 0);
    // drawer.drawLine("velTarget" + std::to_string(ROBOT_ID), myPos, sumOfForces);
    // drawer.setColor(0, 0, 0);

    // Rotate the commands from world frame to robot frame 
    Vector2 velTarget = worldToRobotFrame(sumOfForces, myAngle);

    // @DEBUG info
    Vector2 myVelRobotFrame = worldToRobotFrame(myVel, myAngle);
    roboteam_msgs::WorldRobot myVelRobot;
    myVelRobot.pos.x = myVelRobotFrame.x;
    myVelRobot.pos.y = myVelRobotFrame.y;
    myVelRobot.w = me.w;
    myVelTopic.publish(myVelRobot); 

    // Velocity controller
    // Vector2 velCommand = controller.velocityController(myVelRobotFrame, velTarget);
    Vector2 velCommand = velTarget;

    // Limit angular and linear velocity
    velCommand = controller.limitVel(velCommand);
    angularVelTarget = controller.limitAngularVel(angularVelTarget);

    
    // This may be useful for control purposes: it limits the driving direction when driving forwards-sideways such that it always drives with two wheels and the other
    // wheels remain still
    // double drivingAngle = velCommand.angle();
    // if (drivingAngle >= ((30.0-20.0)/180.0*M_PI) && drivingAngle <= ((30.0+40.0)/180.0*M_PI)) {
    //     ROS_INFO_STREAM("limiting drive direction pos y");
    //     velCommand = Vector2(1.0, 0.0).rotate(30.0/180.0*M_PI).scale(velCommand.length());
    // } else if (drivingAngle >= ((-30.0-40.0)/180.0*M_PI) && drivingAngle <= ((-30.0+20.0)/180.0*M_PI)) {
    //     ROS_INFO_STREAM("limiting drive direction neg y");
    //     velCommand = Vector2(1.0, 0.0).rotate(330.0/180.0*M_PI).scale(velCommand.length());
    // }

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
