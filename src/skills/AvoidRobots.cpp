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
#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_utils/Math.h"
#include "roboteam_tactics/conditions/DistanceXToY.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/world_analysis.h"

namespace rtt {

RTT_REGISTER_SKILL(AvoidRobots);

AvoidRobots::AvoidRobots(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        
        // Rest of the members
        , success(false)
        , maxSpeed(2.0)
        , attractiveForce(10.0)
        , attractiveForceWhenClose(3.0)
        , repulsiveForce(20.0)
        {}

roboteam_msgs::RobotCommand AvoidRobots::stopCommand(uint id) {
    roboteam_msgs::RobotCommand command;
    command.id = id;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;
    return command;
}

// Simple proportional rotation controller
double AvoidRobots::RotationController(double angleError) {
    double pGainRot = 6.0;
    double maxRotSpeed = 3.0;

    if (angleError < M_PI) {angleError += 2*M_PI;}
    if (angleError > M_PI) {angleError -= 2*M_PI;}

    double requiredRotSpeed = angleError * pGainRot;
    if (fabs(requiredRotSpeed) > maxRotSpeed) {
        requiredRotSpeed = requiredRotSpeed / fabs(requiredRotSpeed) * maxRotSpeed;
    }
    return requiredRotSpeed;
}

// TODO: maybe add the controller for the rotational velocity as well, if that appears to be necessary
roboteam_msgs::RobotCommand AvoidRobots::VelocityController(roboteam_utils::Vector2 velTarget, double wTarget, roboteam_utils::Vector2 posError) {
    double velIGain = 0.2;
    // double wIGain = 0.0;

    roboteam_msgs::World world = LastWorld::get();
    roboteam_utils::Vector2 myVel = world.us.at(robotID).vel;
    roboteam_utils::Vector2 velError = velTarget - myVel;
    // double myW = world.us.at(robotID).w;
    // double wError = wTarget - myW;

    double timeStep = 1.0/40.0;
    velControllerI = velControllerI + velError.scale(timeStep);
    // wControllerI = wControllerI + wError * timeStep;
    if (velControllerI.length() > 1.0) {
        velControllerI = velControllerI.scale(1.0 / velControllerI.length());
    }
    if (posError.length() < 0.5) {
        velControllerI = Vector2(0.0, 0.0);
    }
    roboteam_utils::Vector2 velCommand = velTarget + velControllerI * velIGain;
    // double wCommand = wTarget + wControllerI * wIGain;
    double wCommand = wTarget;

    roboteam_utils::Vector2 requiredSpeed;
    double myAngle = world.us.at(robotID).angle;
    requiredSpeed = worldToRobotFrame(velCommand, myAngle);

    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.x_vel = requiredSpeed.x;
    command.y_vel = requiredSpeed.y;
    command.w = wCommand;
    return command;
}

roboteam_utils::Vector2 AvoidRobots::GetForceVectorFromRobot(roboteam_utils::Vector2 myPos, roboteam_utils::Vector2 otherRobotPos, roboteam_utils::Vector2 posError) {
    // Determine how far we should look ahead to avoid other robots
    double lookingDistance = 0.75;
    if (lookingDistance > posError.length()) {
        lookingDistance = posError.length();
    }

    roboteam_utils::Vector2 posDiff = otherRobotPos - myPos;
    roboteam_utils::Vector2 closestPointOnPath = posError.closestPointOnVector(myPos, otherRobotPos);
    roboteam_utils::Vector2 distanceFromPath = otherRobotPos - closestPointOnPath;

    roboteam_utils::Vector2 forceVector(0.0, 0.0);
    if (posDiff.length() < lookingDistance) {
        double scalingNumber1; // scalingNumber1 represents the weight placed on the perpendicular distance between the robot and our path
        if (distanceFromPath.length() < 0.05) {
            scalingNumber1 = 1.0;
        } else if (distanceFromPath.length() > 0.2) {
            scalingNumber1 = 0.0;
        } else {
            scalingNumber1 = 1.0/(distanceFromPath.length()*20);
        }
        double scalingNumber2 = 1/(posDiff.length()*posDiff.length()*20); // scalingNumber2 represents the weight placed on the actual distance between the other robot's pos and our pos
        roboteam_utils::Vector2 distanceFromPathUnit = distanceFromPath.scale(1/distanceFromPath.length());
        forceVector = distanceFromPathUnit.scale(scalingNumber1+scalingNumber2*2);
    }
    return forceVector;
}

roboteam_utils::Vector2 AvoidRobots::CheckTargetPos(roboteam_utils::Vector2 targetPos) {
    double xGoal = targetPos.x;
    double yGoal = targetPos.y;

    double safetyMarginGoalAreas = 0.2;
    double marginOutsideField = 0.2; // meter

    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    if (fabs(yGoal) < (field.goal_width/2 + safetyMarginGoalAreas)) {
        marginOutsideField = 0.0; // we should not go outside the field close to the goal areas.
    }

    if (xGoal > (field.field_length/2+marginOutsideField) || xGoal < (-field.field_length/2-marginOutsideField)) {
        xGoal = signum(xGoal)*(field.field_length/2+marginOutsideField);
    }
    if (yGoal > (field.field_width/2+marginOutsideField) || yGoal < (-field.field_width/2-marginOutsideField)) {
        yGoal = signum(yGoal)*(field.field_width/2+marginOutsideField);
    }

    roboteam_utils::Vector2 newTargetPos(xGoal, yGoal);
    std::string our_side;
    ros::param::get("our_side", our_side);

    roboteam_utils::Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", newTargetPos, safetyMarginGoalAreas);
    roboteam_utils::Vector2 distToTheirDefenseArea = getDistToDefenseArea("their defense area", newTargetPos, safetyMarginGoalAreas);

    if (!GetBool("isKeeper")) {
        if (isWithinDefenseArea("our defense area", newTargetPos)) {
            newTargetPos = newTargetPos + distToOurDefenseArea;
        }

        if (isWithinDefenseArea("their defense area", newTargetPos)) {
            newTargetPos = newTargetPos + distToTheirDefenseArea;
        }
    }
    return newTargetPos;
}

// roboteam_utils::Vector2 AvoidRobots::springDamperForce(roboteam_utils::Vector2 distToOurDefenseArea, roboteam_utils::Vector2 sumOfForces) {
//     roboteam_msgs::World world = LastWorld::get();
//     roboteam_utils::Vector2 myVel(me.vel);
//     roboteam_utils::Vector2 myPos(me.pos);

//     if (distToOurDefenseArea.length() < 1.0) {
//         roboteam_utils::Vector2 shortestDistanceUnit = distToOurDefenseArea.scale(1 / distToOurDefenseArea.length());

//         double dot = shortestDistanceUnit.dot(myVel);
//         roboteam_utils::Vector2 damperForce;
//         if (dot > 0) {
//             double dotUnit = shortestDistanceUnit.dot(myVel.normalize());
//             double factor = 0.0 / (distToOurDefenseArea.length());
//             damperForce = Vector2() - shortestDistanceUnit*sumOfForces.length()*dotUnit*factor;
//             drawer.SetColor(0, 0, 255);
//             drawer.DrawLine("damping", myPos, damperForce);
//         } else {
//             drawer.RemoveLine("damping");
//         }

//         roboteam_utils::Vector2 springForce;
//         if (distToOurDefenseArea.length() < 1.0) {
//             double factor = 10 / (distToOurDefenseArea.length() * distToOurDefenseArea.length());
//             springForce = Vector2() - shortestDistanceUnit.scale(factor);
//             drawer.SetColor(255, 0, 0);
//             drawer.DrawLine("spring", myPos, springForce);
//         } else {
//             drawer.RemoveLine("spring");
//         }

//         return damperForce + springForce;
//     }
//     return Vector2();
// }

bt::Node::Status AvoidRobots::Update () {
    // Get the latest world state
	roboteam_msgs::World world = LastWorld::get();
    if (world.us.size() == 0) {
        ROS_INFO("No information about the world state :(");
        return Status::Running;
    }

    // Get blackboard info
    xGoal = GetDouble("xGoal");
    yGoal = GetDouble("yGoal");
    angleGoal = GetDouble("angleGoal");
    robotID = blackboard->GetInt("ROBOT_ID");
    dribbler = GetBool("dribbler");

    boost::optional<roboteam_msgs::WorldRobot> findBot = lookup_bot(robotID, true, &world);
    if (findBot) {
        me = *findBot;
    } else {
        ROS_WARN("AvoidRobots: robot with this ID not found");
    }

    // Checking inputs
    roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(xGoal, yGoal);
    drawer.DrawPoint("targetPos", targetPos);
    targetPos = CheckTargetPos(targetPos); // TODO: this does not need to be done every Update, only when the goal position changes
    drawer.SetColor(0, 0, 255);
    drawer.DrawPoint("newTargetPos", targetPos);
    drawer.SetColor(0, 0, 0);
    angleGoal = cleanAngle(angleGoal);

    roboteam_utils::Vector2 myPos(me.pos);
    roboteam_utils::Vector2 myVel(me.vel);
    roboteam_utils::Vector2 posError = targetPos - myPos;
    double myAngle = me.angle;
    if (posError.length() > 0.8) {
        angleGoal = posError.angle();
    }
    double angleError = angleGoal - myAngle;

    auto bb2 = std::make_shared<bt::Blackboard>();
    bb2->SetInt("me", robotID);
    bb2->SetDouble("x_coor", xGoal);
    bb2->SetDouble("y_coor", yGoal);
    bb2->SetBool("check_move", true);

    // Get global robot command publisher
    auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();

    // For each robot, add the repulsive force it exerts on us
    roboteam_utils::Vector2 sumOfForces(0.0, 0.0);

    for (size_t i = 0; i < world.us.size(); i++) {
        if (i != robotID) { // TODO: change this to check whether the robot ID corresponds to i instead of just its place in world.us
            roboteam_utils::Vector2 otherRobotPos = roboteam_utils::Vector2(world.us.at(i).pos.x, world.us.at(i).pos.y);
            roboteam_utils::Vector2 forceVector = GetForceVectorFromRobot(myPos, otherRobotPos, posError);
            sumOfForces = sumOfForces - forceVector*repulsiveForce;
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_utils::Vector2 otherRobotPos = roboteam_utils::Vector2(world.them.at(i).pos.x, world.them.at(i).pos.y);
        roboteam_utils::Vector2 forceVector = GetForceVectorFromRobot(myPos, otherRobotPos, posError);
        sumOfForces = sumOfForces - forceVector*repulsiveForce;
    }

    // Add an attractive force towards the target
    sumOfForces = sumOfForces + posError*attractiveForceWhenClose;

    // Slow down once we get close to the goal, other go at maximum speed
    if (posError.length() > 1.0) {
        if (sumOfForces.length() > 0) {
            sumOfForces = sumOfForces.scale(1/sumOfForces.length() * maxSpeed);
        } else {
            sumOfForces = roboteam_utils::Vector2(0.0, 0.0);
        }
    }

    // Avoid the goal areas by moving only parralel to the goal area when close enough
    roboteam_utils::Vector2 distToOurDefenseArea = getDistToDefenseArea("our defense area", myPos, 0.2);
    if (fabs(distToOurDefenseArea.length() < 0.5) && posError.length() > 0.5 && myVel.dot(distToOurDefenseArea) > 0) {
        if (sumOfForces.dot(distToOurDefenseArea.rotate(0.5*M_PI)) > 0) {
            sumOfForces = distToOurDefenseArea.rotate(0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
        } else {
            sumOfForces = distToOurDefenseArea.rotate(-0.5*M_PI).scale(sumOfForces.length() / distToOurDefenseArea.length());
        }
    }

    // Draw the velocity vector acting on the robots
    // drawer.DrawLine("sumOfForces", myPos, sumOfForces);

    if (posError.length() < 0.01 && fabs(angleError) < 0.1) {
        roboteam_msgs::RobotCommand command = stopCommand(robotID);
        pub.publish(command);
        return Status::Success;
    }

    // Rotate from robot frame to world frame
    double requiredRotSpeed = RotationController(angleError);
    roboteam_msgs::RobotCommand command = VelocityController(sumOfForces, requiredRotSpeed, posError);
    if (dribbler) {command.dribbler = true;}
    pub.publish(command);
    return Status::Running;
};

} // rtt
