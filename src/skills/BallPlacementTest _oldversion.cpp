#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <ros/ros.h>
#include <cmath>

#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GoToPos.h"
#include "roboteam_tactics/skills/BallPlacementTest_oldversion.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/LastRef.h"
#include "roboteam_tactics/utils/ScopedBB.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_tactics/conditions/IsInDefenseArea.h"

#define RTT_CURRENT_DEBUG_TAG BallPlacementTest

namespace rtt {

RTT_REGISTER_SKILL(BallPlacementTest);

BallPlacementTest::BallPlacementTest(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , goToPosObj("", private_bb)

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
            avoidRobotsGain = 0.15;

        }


void BallPlacementTest::sendStopCommand(uint id) {
    roboteam_msgs::RobotCommand command = controller.getStopCommand(id);

    // Get global robot command publisher, and publish the command
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
    pub.publish(command);
}


// Used in the avoidRobots function. Computes a virtual repelling 'force' that each other robot exerts on our robot, in order to avoid them7
// Based on this algorithm: https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
Vector2 BallPlacementTest::getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna, Vector2 targetPos) {

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
Vector2 BallPlacementTest::avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos) {
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
    // drawer.setColor(255, 0, 0);
    // drawer.drawLine("antenna", myPos, antenna);

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

    drawer.setColor(255, 0, 0);
    drawer.drawLine("sumOfForces", myPos, sumOfForces);
    drawer.setColor(0, 0, 0);

    return sumOfForces;
}


// Computes a velocity vector that can be added to the normal velocity command vector, in order to avoid the goal areas by moving only parralel to the goal area when close
Vector2 BallPlacementTest::avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces) {
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


Vector2 BallPlacementTest::avoidBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces, Vector2 targetPos, Vector2 myVel) {
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
Vector2 BallPlacementTest::checkTargetPos(Vector2 targetPos) {
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();

    double xGoal = targetPos.x;
    double yGoal = targetPos.y;

    // We should not go outside the field close to the goal areas
    if (fabs(yGoal) < (field.goal_width/2 + safetyMarginGoalAreas)) {
        marginOutsideField = -0.1;
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


boost::optional<roboteam_msgs::RobotCommand> BallPlacementTest::getVelCommand() {

    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
//    Vector2 targetPos = LastRef::get().designated_position;
//    rtt::ScopedBB(bb, "BallPlacementTest")
//            .setDouble("xGoal", targetPos.x)
//            .setDouble("yGoal", targetPos.y)
//            ;
    Vector2 targetPos = Vector2(GetDouble("xGoal"), GetDouble("yGoal"));
    std::cout << "xGoal: " << targetPos.x << ", yGoal: " << targetPos.y << std::endl;
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

//ballplacement
    if (HasBool("ballPlacement")) {

        controller.setControlParam("maxSpeed", 0.1);



        // Get the latest world state
        roboteam_msgs::World world = LastWorld::get();

        // Find the robot with the specified ID
        boost::optional <roboteam_msgs::WorldRobot> findBot = getWorldBot(ROBOT_ID);

        roboteam_msgs::WorldBall ball = world.ball;
        roboteam_msgs::WorldRobot me;

        if (findBot) {
            me = *findBot;
        } else {
            ROS_WARN_STREAM("BallPlacementTest: robot with this ID not found, ID: " << ROBOT_ID);
            failure = true;
            succeeded = false;
            return boost::none;
        }

        // Store some variables for easy access
        Vector2 myPos(me.pos);
        Vector2 myVel(me.vel);


        Vector2 ballPos = Vector2(ball.pos.x, ball.pos.y);


        Vector2 ballPosError = targetPos - ballPos;


        Vector2 robotTargetPos = myPos + ballPosError;

        Vector2 robotBallError = myPos - ballPos;

        if (robotBallError.length() > 0.5){
            ROS_WARN_STREAM("BallPlacementTest: robot " << ROBOT_ID << " lost ball");
            sendStopCommand(ROBOT_ID);
            failure = true;
            succeeded = false;
            return boost::none;
        }



        // Draw the line towards the target position
        drawer.setColor(0, 100, 100);
        drawer.drawLine("posError_" + std::to_string(ROBOT_ID), myPos, ballPosError);
        // drawer.drawPoint("targetPos_" + std::to_string(ROBOT_ID), targetPos);
        drawer.setColor(0, 0, 0);

        double angleGoal;
        angleGoal = ballPosError.angle() + M_PI;

        double myAngle = me.angle;
        double angleError = cleanAngle(angleGoal - myAngle);
        double myAngularVel = me.w;

        // Determine how close we should get to the targetPos before we succeed
        double successDist;
        if (HasDouble("successDist")) {
            successDist = GetDouble("successDist");
        } else {
            successDist = 0.06;
        }

        // If we are close enough to our target position and target orientation, then stop the robot and return success
        if (ballPosError.length() < successDist){
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
        sumOfForces = sumOfForces + controller.positionController(myPos, robotTargetPos, myVel);

        // Rotate the commands from world frame to robot frame
        Vector2 velTarget = worldToRobotFrame(sumOfForces, myAngle);


        // Velocity controller
        // Vector2 velCommand = controller.velocityController(myVelRobotFrame, velTarget);
        Vector2 velCommand = velTarget;
        std::cout << "velCommand - x: " << velCommand.x << "y: " << velCommand.y << std::endl;

        // Limit angular and linear velocity
        velCommand = controller.limitVel(velCommand, angleGoal);
        std::cout << "limit velCommand - x: " << velCommand.x << "y: " << velCommand.y << std::endl;


        // Fill the command message
        roboteam_msgs::RobotCommand command;
        command.id = ROBOT_ID;
        command.x_vel = velCommand.x;
        command.y_vel = velCommand.y;
        command.w = 0;
//        if (succeeded == true){
//            command.dribbler = false;
//        }
//        else {
            command.dribbler = true;
//        }

        return command;


    } //end ballplacement
}

bt::Node::Status BallPlacementTest::Update() {

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
