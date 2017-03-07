#include "roboteam_tactics/treegen/LeafRegister.h"
#include <string>
#include <vector>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPosAlt.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Cone.h"

namespace rtt {

RTT_REGISTER_SKILL(GoToPosAlt);

GoToPosAlt::GoToPosAlt(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        
        // Rest of the members
        , success(false)
        , maxSpeed(1.0)
        , attractiveForce(10.0)
        , attractiveForceWhenClose(10.0)
        , repulsiveForce(20.0)
        , safetyMarginGoalAreas(0.2)
        , marginOutsideField(0.2)
        , angleErrorInt(0.0)

        // Control gains
        , pGainPosition(3.0)
        , pGainRotation(5.0)
        // , iGainRotation(0.5)
        // , dGainRotation(0.2)
        , maxAngularVel(3.0)
        , iGainVelocity(0.5)
        , iGainAngularVel(0.02)

        {
            print_blackboard(blackboard);
            start = now();
            angleErrorHistory = (double*) calloc(10,sizeof(double));
        }


void GoToPosAlt::sendStopCommand(uint id) {

    roboteam_msgs::RobotCommand command;
    command.id = id;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;
    if (GetBool("dribbler")) {
        command.dribbler = true;
    } else {
        command.dribbler = false;
    }

    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}


// Proportional position controller
Vector2 GoToPosAlt::positionController(Vector2 myPos, Vector2 targetPos) {
    Vector2 posError = targetPos - myPos;
    Vector2 velTarget = posError*attractiveForceWhenClose;
    return velTarget;
}


// Proportional rotation controller
double GoToPosAlt::rotationController(double myAngle, double angleGoal, Vector2 posError) {

    // Proportional term
    double angleError = angleGoal - myAngle;
    angleError = cleanAngle(angleError);

    // // Integral term
    // double updateRate;
    // ros::param::get("role_iterations_per_second", updateRate);
    // double timeStep = 1 / updateRate;
    // angleErrorInt += angleError * timeStep;
    
    // // Differential term (UNTESTED!)
    // angleErrorHistory[historyIndex] = angleError;
    // int amountOfHistories = 4;
    // historyIndex = (historyIndex + 1) % amountOfHistories;
    // double avgHistory = 0;
    // for (int i = 0; i < amountOfHistories; i++) {
    //     avgHistory += angleErrorHistory[i];
    // }
    // avgHistory /= ((double) amountOfHistories);
    // double angleErrorDiff = avgHistory * timeStep;

    // Maybe we later choose to add an integral and/or differential term to this controller if it proves to be necessary, the basic structure
    // for this is already there, although it is not yet well tested
    double angularVelTarget = angleError * pGainRotation;    

    // Limit the angular velocity target
    if (fabs(angularVelTarget) > maxAngularVel) {
        angularVelTarget = angularVelTarget / fabs(angularVelTarget) * maxAngularVel;
    }

    return angularVelTarget;
}


// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them
double GoToPosAlt::getAngleFromRobot(Vector2 myPos, Vector2 otherRobotPos, double lookingDistance, Cone antennaCone) {
    Vector2 antenna = antennaCone.center - myPos;    

    double angle;
    if ((otherRobotPos-myPos).length() < lookingDistance && antennaCone.IsWithinCone(otherRobotPos)) {
        // double distanceToCenter = (otherRobotPos - antenna.closestPointOnVector(myPos, otherRobotPos)).length();
        if (isBetweenAngles(antenna.angle(), antennaCone.side1.angle(), (otherRobotPos - antennaCone.start).angle())) {
            angle = antenna.angle()-0.1*M_PI;
        }
        if (isBetweenAngles(antennaCone.side2.angle(), antenna.angle(), (otherRobotPos - antennaCone.start).angle())) {
            angle = antenna.angle()+0.1*M_PI;
        }
    }    
    return angle;
}


// Computes an angle for the robot to avoid other robots, assumes robot will ride forward
double GoToPosAlt::avoidRobotsForward(Vector2 myPos, Vector2 myVel, Vector2 targetPos){

    roboteam_msgs::World world = LastWorld::get();

    Vector2 posError = targetPos - myPos;
    double lookingDistance = 0.7; // default
    if (lookingDistance > posError.length()) {
        lookingDistance = posError.length();
    }
    
    Vector2 antenna = Vector2(lookingDistance, 0.0).rotate(posError.angle());
    Vector2 coneStart = myPos - antenna;
    Cone antennaCone(coneStart, (antenna + myPos), 0.3);

    // Draw the lines of the cone in rqt_view
    Vector2 coneSide1 = (antennaCone.center-antennaCone.start).rotate(0.5*antennaCone.angle);
    Vector2 coneSide2 = (antennaCone.center-antennaCone.start).rotate(-0.5*antennaCone.angle);
    drawer.DrawLine("coneRobotsSide1", antennaCone.start, coneSide1);
    drawer.DrawLine("coneRobotsSide2", antennaCone.start, coneSide2);

    Vector2 sumOfForces;
    double sumOfAngles;

    for (size_t i = 0; i < world.us.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = world.us.at(i);
        if (currentRobot.id != ROBOT_ID) {

            Vector2 otherRobotPos(currentRobot.pos);
            double angle = getAngleFromRobot(myPos, otherRobotPos, lookingDistance, antennaCone);
            sumOfAngles = sumOfAngles + angle;
        }
    }

    for (size_t i = 0; i < world.them.size(); i++) {
        Vector2 otherRobotPos(world.them.at(i).pos);
        double angle = getAngleFromRobot(myPos, otherRobotPos, lookingDistance, antennaCone);
        sumOfAngles = sumOfAngles + angle; 
    }

    return sumOfAngles;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid the goal areas by moving only parralel to the goal area when close
Vector2 GoToPosAlt::avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces) {
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


// Makes sure that the given target position is not inside a goal area, or (too far) outside the field. "Too far" is specified by the class variable marginOutsideField
Vector2 GoToPosAlt::checkTargetPos(Vector2 targetPos) {
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

    double xGoal = targetPos.x;
    double yGoal = targetPos.y;

    // We should not go outside the field close to the goal areas
    if (fabs(yGoal) < (field.goal_width/2 + safetyMarginGoalAreas)) {
        marginOutsideField = 0.0; 
    }

    // If the target position is outside of the field + margins, then change the target position to the closest point within this margin
    if (xGoal > (field.field_length/2+marginOutsideField) || xGoal < (-field.field_length/2-marginOutsideField)) {
        xGoal = signum(xGoal)*(field.field_length/2+marginOutsideField);
    }
    if (yGoal > (field.field_width/2+marginOutsideField) || yGoal < (-field.field_width/2-marginOutsideField)) {
        yGoal = signum(yGoal)*(field.field_width/2+marginOutsideField);
    }

    Vector2 newTargetPos(xGoal, yGoal);

    // If the current robot is not a keeper, we should take into account that it cannot enter the defense area
    if (ROBOT_ID != KEEPER_ID) {
        
        // If the target position is in our defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea("our defense area", newTargetPos)) {
            Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", newTargetPos, safetyMarginGoalAreas);
            newTargetPos = newTargetPos + distToOurDefenseArea;
        }

        // If the target position is in their defense area, then subtract the vector difference between the defense area and the target position
        if (isWithinDefenseArea("their defense area", newTargetPos)) {
            Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", newTargetPos, safetyMarginGoalAreas);
            newTargetPos = newTargetPos + distToTheirDefenseArea;
        }
    }

    return newTargetPos;
}


roboteam_msgs::RobotCommand GoToPosAlt::getVelCommand() {

    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();
    

    // Get blackboard info
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    angleGoal = cleanAngle(GetDouble("angleGoal"));  
    if (blackboard->HasInt("KEEPER_ID")) {
        KEEPER_ID = blackboard->GetInt("KEEPER_ID");
    } else {
        // ROS_WARN("GoToPosAlt, KEEPER_ID not set");
        KEEPER_ID = 100;
    }
    

    // Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = lookup_bot(ROBOT_ID, true, &world);
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN_STREAM("GoToPosAlt: robot with this ID not found, ID: " << ROBOT_ID);
    }


    // Checking inputs
    if (targetPos == prevTargetPos) {
        targetPos = prevTargetPos;
    } else {
        targetPos = checkTargetPos(targetPos);
        prevTargetPos = targetPos;
    }


    // Draw the target position in RQT-view
    drawer.SetColor(0, 0, 0);
    drawer.DrawPoint("targetPos", targetPos);


    // Store some variables for easy access
    Vector2 myPos(me.pos);
    Vector2 myVel(me.vel);
    Vector2 posError = targetPos - myPos;
    double myAngle = me.angle;
    double angleError = angleGoal - myAngle;
    


    // If we are close to the target we can also drive backwards if that's easier
    bool driveBackwards = false;
    if (posError.length() < 1.0) {
        if (fabs(cleanAngle(cleanAngle(posError.angle() + M_PI) - myAngle)) < 0.5) {
            angleGoal = cleanAngle(posError.angle() + M_PI);
            angleError = cleanAngle(angleGoal - myAngle);
            driveBackwards = true;
        }
    }


    // @HACK: Make oh so smooth qualification turns
    if (HasDouble("angleGoal") && !driveBackwards) {
        if (fabs(angleError) > 0.2 || fabs(posError.angle()-myAngle) > 0.2 || posError.length() > 0.8) {
            // double endAngleGoal = GetDouble("angleGoal");
            double angleToTargetPos = posError.angle();
            double angleDiff = cleanAngle(angleGoal - angleToTargetPos);
            
            double distanceFromTarget = posError.length()*0.75;
            if (distanceFromTarget > 1.0) distanceFromTarget = 1.0;
            
            Vector2 firstStop = Vector2(distanceFromTarget, 0.0).rotate(cleanAngle(angleGoal+M_PI));
            firstStop = firstStop.rotate(-0.8*angleDiff);
            firstStop = firstStop + targetPos;
            targetPos = firstStop;
            posError = targetPos - myPos;

            // Draw the target position in RQT-view
            drawer.SetColor(255, 255, 0);
            drawer.DrawPoint("targetPos2", targetPos);
        } else {
            drawer.RemovePoint("targetPos2");
        }
    }


    // Always face towards the target position
    angleGoal = posError.angle();
    angleError = angleGoal - myAngle;


    // A vector to combine all the influences of different controllers (normal position controller, obstacle avoidance, defense area avoidance...)
    Vector2 sumOfForces(0.0, 0.0);


    // Position controller to steer the robot towards the target position
    sumOfForces = sumOfForces + positionController(myPos, targetPos);


    // Robot avoidance
    if (HasBool("avoidRobots")) {
        if (GetBool("avoidRobots")) {
            double avoidAngle = avoidRobotsForward(myPos, myVel, targetPos);
            angleGoal=angleGoal+avoidAngle;
        }
    } else {
        // ROS_WARN("You did not set the boolean avoidRobots in GoToPosAlt");
    }


    // Defense area avoidance
    // sumOfForces = avoidDefenseAreas(myPos, myVel, targetPos, sumOfForces);


    // Rotation controller to make sure the robot has and keeps the correct orientation
    double angularVelTarget = rotationController(myAngle, angleGoal, posError);


    // Limit the robot velocity to the maximum speed, but also ensure that it goes at maximum speed when not yet close to the target. Because 
    // it might the case that an opponent is blocking our robot, and its sumOfForces is therefore low, but since it is far away from the target
    // it should still go at maximum speed and use the sumOfForces vector only for direction.
    if (posError.length() > 1.0 || sumOfForces.length() > maxSpeed) {
        if (sumOfForces.length() > 0) {
            sumOfForces = sumOfForces.scale(1/sumOfForces.length() * maxSpeed);
        } else {
            sumOfForces = Vector2(0.0, 0.0);
        }
    }


    // For now, only drive forward in combination with an angular velocity
    double driveSpeed = maxSpeed;
    if (fabs(angleError) > 1.0) {
        driveSpeed = 1.0 / (fabs(angleError)) * maxSpeed;
    }


    Vector2 velCommand;


    // TODO: @Hack Maybe remove this at some point
    auto mode = getMode();
    if (mode == Mode::GRSIM) { 
        if (driveBackwards) {
            velCommand = Vector2(-1.0, 0.0).scale(driveSpeed);
        } else {
            velCommand = Vector2(1.0, 0.0).scale(driveSpeed);
        }
    } else {
        if (driveBackwards) {
            velCommand = Vector2(0.0, -1.0).scale(driveSpeed);
        } else {
            velCommand = Vector2(0.0, 1.0).scale(driveSpeed);
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


bt::Node::Status GoToPosAlt::Update() {

    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();
    if (world.us.size() == 0) {
        ROS_INFO("No information about the world state :(");
        return Status::Running;
    }


    // Find the robot with the specified ID
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    boost::optional<roboteam_msgs::WorldRobot> findBot = lookup_bot(ROBOT_ID, true, &world);
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN_STREAM("GoToPosAlt: robot with this ID not found, ID: " << ROBOT_ID);
    }


    // Get Blackboard info
    Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    angleGoal = cleanAngle(GetDouble("angleGoal"));
    

    // Store some variables for easy access
    Vector2 myPos(me.pos);
    Vector2 posError = targetPos - myPos;
    double myAngle = me.angle;
    double angleError = angleGoal - myAngle;


    // If we are close enough to our target position and target orientation, then stop the robot and return success
    if (posError.length() < 0.2 && fabs(angleError) < 0.2) {
        sendStopCommand(ROBOT_ID);
        return Status::Success;
    }

    roboteam_msgs::RobotCommand command = getVelCommand();
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
    return Status::Running;
}

} // rtt