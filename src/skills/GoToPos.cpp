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
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Cone.h"

namespace rtt {

RTT_REGISTER_SKILL(GoToPos);

GoToPos::GoToPos(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , success(false)

        // Control gains
        , pGainPosition(3.0)
        , pGainRotation(3.0) // was 5?
        // , iGainRotation(0.5)
        // , dGainRotation(0.2)
        , maxAngularVel(3.0)
        , minAngularVel(0.0)
        , iGainVelocity(0.5)
        , iGainAngularVel(0.02)
        
        // Rest of the members
        , maxSpeed(1.0)
        , minSpeed(0.0)
        , attractiveForce(10.0)
        , attractiveForceWhenClose(2.0) // was 5? 
        , repulsiveForce(20.0)
        , safetyMarginGoalAreas(0.2)
        , marginOutsideField(1.2)
        , angleErrorIntegral(0.0)
        , historyIndex(0)
        
        {
            start = now();
            angleErrorHistory = (double*) calloc(10,sizeof(double));
            succeeded = false;
        }


void GoToPos::sendStopCommand(uint id) {

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
Vector2 GoToPos::positionController(Vector2 myPos, Vector2 targetPos) {
    Vector2 posError = targetPos - myPos;
    Vector2 velTarget = posError*attractiveForceWhenClose;
    return velTarget;
}


// Proportional rotation controller
double GoToPos::rotationController(double myAngle, double angleGoal, Vector2 posError) {

    if (posError.length() > 1.0) {
        angleGoal = posError.angle();
    }

    // Proportional term
    double angleError = angleGoal - myAngle;
    angleError = cleanAngle(angleError);

    // // Integral term
    // double updateRate;
    // ros::param::get("role_iterations_per_second", updateRate);
    // double timeStep = 1 / updateRate;
    // angleErrorIntegral += angleError * timeStep;
    
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
// TODO: fix the problem that if there are two robots in the way on both sides of the cone, our robot tries to push through them regardless
// of whether he hits them or not (maybe use the commented variable distanceToCenter to accomplish this)
Vector2 GoToPos::getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, double lookingDistance, Cone antennaCone) {
    Vector2 antenna = antennaCone.center - myPos;    

    Vector2 forceVector(0.0, 0.0);
    if ((otherRobotPos-myPos).length() < lookingDistance && antennaCone.IsWithinCone(otherRobotPos)) {
        // double distanceToCenter = (otherRobotPos - antenna.closestPointOnVector(myPos, otherRobotPos)).length();
        if (isBetweenAngles(antenna.angle(), antennaCone.side1.angle(), (otherRobotPos - antennaCone.start).angle())) {
            forceVector = antenna.rotate(-0.5*M_PI).scale(2 / (otherRobotPos - myPos).length());
        }
        if (isBetweenAngles(antennaCone.side2.angle(), antenna.angle(), (otherRobotPos - antennaCone.start).angle())) {
            forceVector = antenna.rotate(0.5*M_PI).scale(2 / (otherRobotPos - myPos).length());
        }
    }    
    return forceVector;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid crashing into other robots
Vector2 GoToPos::avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos) {
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
    drawer.drawLine("coneGoToPosSide1", antennaCone.start, coneSide1);
    drawer.drawLine("coneGoToPosSide2", antennaCone.start, coneSide2);

    Vector2 sumOfForces;
    for (size_t i = 0; i < world.us.size(); i++) {
        roboteam_msgs::WorldRobot currentRobot = *getWorldBot(i);
        if (currentRobot.id != ROBOT_ID) {

            Vector2 otherRobotPos(currentRobot.pos);
            Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, lookingDistance, antennaCone);
            sumOfForces = sumOfForces + forceVector;
        }
    }

    for (size_t i = 0; i < world.them.size(); i++) {
        Vector2 otherRobotPos(world.them.at(i).pos);
        Vector2 forceVector = getForceVectorFromRobot(myPos, otherRobotPos, lookingDistance, antennaCone);
        sumOfForces = sumOfForces + forceVector; 
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
    if (xGoal > (field.field_length/2+marginOutsideField) || xGoal < (-field.field_length/2-marginOutsideField)) {
        xGoal = signum(xGoal)*(field.field_length/2+marginOutsideField);
    }
    if (yGoal > (field.field_width/2+marginOutsideField) || yGoal < (-field.field_width/2-marginOutsideField)) {
        yGoal = signum(yGoal)*(field.field_width/2+marginOutsideField);
    }

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

    return newTargetPos;
}

boost::optional<roboteam_msgs::RobotCommand> GoToPos::getVelCommand() {
    // Get the latest world state
    roboteam_msgs::World world = LastWorld::get();

    if (HasDouble("maxSpeed")) {
        maxSpeed = GetDouble("maxSpeed");
    }
    
    if (HasDouble("minSpeed")) {
        minSpeed = GetDouble("minSpeed");
    }

    if (HasDouble("minAngularVel")) {
        minSpeed = GetDouble("minAngularVel");
    }

    // Get blackboard info
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));

    angleGoal = cleanAngle(GetDouble("angleGoal"));  
    if (blackboard->HasInt("KEEPER_ID")) {
        KEEPER_ID = blackboard->GetInt("KEEPER_ID");
    } else {
        // ROS_WARN("GoToPos, KEEPER_ID not set");
        KEEPER_ID = 100;
    }

    // Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(ROBOT_ID);
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN_STREAM("GoToPos: robot with this ID not found, ID: " << ROBOT_ID);
        return boost::none;
    }  

    // Checking inputs
    if (targetPos == prevTargetPos) {
        targetPos = prevTargetPos;
    } else {
        targetPos = checkTargetPos(targetPos);
        prevTargetPos = targetPos;
    }

    // Draw the target position in RQT-view
    drawer.setColor(0, 0, 0);
    drawer.drawPoint("targetPos_" + std::to_string(ROBOT_ID), targetPos);

    // Store some variables for easy access
    Vector2 myPos(me.pos);
    Vector2 myVel(me.vel);
    Vector2 posError = targetPos - myPos;
    double myAngle = me.angle;
    double angleError = angleGoal - myAngle;

    // If we are close enough to our target position and target orientation, then stop the robot and return success
    if (posError.length() < 0.01 && fabs(angleError) < 0.05) {
        sendStopCommand(ROBOT_ID);
        succeeded = true;
        roboteam_msgs::RobotCommand command;
        return boost::none;
    }

    // A vector to combine all the influences of different controllers (normal position controller, obstacle avoidance, defense area avoidance...)
    Vector2 sumOfForces(0.0, 0.0);

    // Position controller to steer the robot towards the target position
    sumOfForces = sumOfForces + positionController(myPos, targetPos);

    // Robot avoidance
    if (HasBool("avoidRobots")) {
        if (GetBool("avoidRobots")) {
            sumOfForces = sumOfForces + avoidRobots(myPos, myVel, targetPos);
        }
    } 

    // if (HasBool("avoidBall")) {
        // std::cout << "Avoiding ball!\n";
        // roboteam_msgs::WorldBall ball = world.ball;

        // Vector2 ballPos(ball.pos);
        // Vector2 diff = ballPos - myPos;

        // double theta = fabs(cleanAngle(diff.angle() - sumOfForces.angle()));

        // std::cout << "Theta: " << theta / M_PI * 180 << "\n";

        // if (theta < (0.5 * M_PI)) {
            // if (theta == 0) theta = 0.01;

            // double force = theta / (0.5 * M_PI);

            // auto projectedBall = ballPos.project(myPos, myPos + sumOfForces);
            // auto ballForce = projectedBall - ballPos;

            // std::cout << "Ballforce: " << ballForce << "\n";
            // sumOfForces = sumOfForces + ballForce * 5;
        // }
    // }

    // Defense area avoidance
    sumOfForces = avoidDefenseAreas(myPos, myVel, targetPos, sumOfForces);

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

    if (sumOfForces.length() < minSpeed) {
        sumOfForces = Vector2(0.0, 0.0);
    }

    if (angularVelTarget < minAngularVel) {
        angularVelTarget = 0;
    }

    Vector2 velCommand = sumOfForces;
    // Draw the velocity vector acting on the robots
    drawer.drawLine("velCommand", myPos, velCommand);  
    // Rotate the commands from world frame to robot frame 
    velCommand = worldToRobotFrame(velCommand, myAngle);

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

    // Maybe not the best way?? Because it is harder to take into account failure in getVelCommand() this way...
    boost::optional<roboteam_msgs::RobotCommand> command = getVelCommand();
    if (command) {
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        pub.publish(*command);
        return Status::Running;
    } else {
        return Status::Success;
    }
}

} // rtt
