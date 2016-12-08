#include "roboteam_tactics/treegen/LeafRegister.h"
#include <string>
#include <vector>

#include "ros/ros.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include "roboteam_tactics/utils/Math.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

RTT_REGISTER_SKILL(AvoidRobots);

AvoidRobots::AvoidRobots(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard) {
	pub = n.advertise<roboteam_msgs::RobotCommand>(TOPIC_COMMANDS, 1000);
}

// Simple proportional rotation controller
double AvoidRobots::RotationController(double angleError) {
    double pGainRot = 3.0;
    double maxRotSpeed = 3.0;

    if (angleError < M_PI) {angleError += 2*M_PI;}
    if (angleError > M_PI) {angleError -= 2*M_PI;}

    double requiredRotSpeed = angleError * pGainRot;
    if (fabs(requiredRotSpeed) > maxRotSpeed) {
        requiredRotSpeed = requiredRotSpeed / fabs(requiredRotSpeed) * maxRotSpeed;
    }
    return requiredRotSpeed;
}

roboteam_msgs::RobotCommand AvoidRobots::PositionController(roboteam_utils::Vector2 posError, double angleError, double myAngle) {

    double requiredRotSpeed = RotationController(angleError);
    roboteam_utils::Vector2 forceVector = posError*attractiveForceWhenClose;

    // Slow down once we get close to the goal, otherwise go at maximum speed
    if (posError.length() > 0.5) { // TODO: compute this distance depending on the maximum speed, so that there is no overshoot
        if (forceVector.length() > 0) {
            forceVector = forceVector.scale(1/forceVector.length() * maxSpeed);
        } else {
            forceVector = roboteam_utils::Vector2(0.0, 0.0);
        }
    }
    
    // Rotate from robot frame to world frame
    roboteam_utils::Vector2 requiredSpeed;  
    requiredSpeed.x=forceVector.x*cos(-myAngle)-forceVector.y*sin(-myAngle);
    requiredSpeed.y=forceVector.x*sin(-myAngle)+forceVector.y*cos(-myAngle);

    if (posError.length() < 0.01 && fabs(angleError) < 0.05) {
        roboteam_msgs::RobotCommand command;
        command.id = robotID;
        command.x_vel = 0.0;
        command.y_vel = 0.0;
        command.w = 0.0;
        if (dribbler) {command.dribbler = true;}
        success = true;
        return command;
    } else {
        roboteam_msgs::RobotCommand command;
        command.id = robotID;
        command.x_vel = requiredSpeed.x;
        command.y_vel = requiredSpeed.y;
        command.w = requiredRotSpeed;
        if (dribbler) {command.dribbler = true;}
        return command;
    }
}

//TODO
roboteam_msgs::RobotCommand AvoidRobots::VelocityController() {
    roboteam_msgs::RobotCommand r;
    return r;
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
    double marginOutsideField = 0.5; // meter
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    if (xGoal > field.field_length/2+marginOutsideField || xGoal < -field.field_length/2-marginOutsideField) {
        xGoal = signum(xGoal)*field.field_length/2+marginOutsideField;
        ROS_WARN("position target outside of field");
    }
    if (yGoal > field.field_width/2+marginOutsideField || yGoal < -field.field_width/2-marginOutsideField) {
        yGoal = signum(yGoal)*field.field_width/2-marginOutsideField;
        ROS_WARN("position target outside of field");
    }

    //TODO: check whether the target position is not within the goal area where we are not allowed to come

    roboteam_utils::Vector2 newTargetPos(xGoal, yGoal);
    return newTargetPos;
}

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

    // Checking inputs
    roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(xGoal, yGoal);
    targetPos = CheckTargetPos(targetPos);
    angleGoal = cleanAngle(angleGoal);
    
    roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us.at(robotID).pos.x, world.us.at(robotID).pos.y);
    roboteam_utils::Vector2 posError = targetPos - myPos;
    double myAngle = world.us.at(robotID).angle;
    if (posError.length() > 1.5) {
        angleGoal = posError.angle();
    }
    double angleError = angleGoal - myAngle;

    auto bb2 = std::make_shared<bt::Blackboard>();
    bb2->SetInt("me", robotID);
    bb2->SetDouble("x_coor", xGoal);
    bb2->SetDouble("y_coor", yGoal);
    bb2->SetBool("check_move", true);
    

    // If you can see the end point, just go towards it
    CanSeePoint canSeePoint("", bb2);
    if (canSeePoint.Update() == Status::Success) {
        roboteam_msgs::RobotCommand command = PositionController(posError, angleError, myAngle);
        pub.publish(command);
        if (success) {
            return Status::Success;
        } else {
            return Status::Running;
        }
    }

    // For each robot, compute the 'force' it exerts on us, and add these forces
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
    if (posError.length() > 0.2) {
        if (sumOfForces.length() > 0) {
            sumOfForces = sumOfForces.scale(1/sumOfForces.length() * maxSpeed);
        } else {
            sumOfForces = roboteam_utils::Vector2(0.0, 0.0);
        }
    }


    // Rotate from robot frame to world frame
    roboteam_utils::Vector2 requiredSpeed;  
    requiredSpeed.x=sumOfForces.x*cos(-myAngle)-sumOfForces.y*sin(-myAngle);
    requiredSpeed.y=sumOfForces.x*sin(-myAngle)+sumOfForces.y*cos(-myAngle);

    double requiredRotSpeed = RotationController(angleError);

    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.x_vel = requiredSpeed.x;
    command.y_vel = requiredSpeed.y;
    command.w = requiredRotSpeed;
    // command.x_vel = 0.0;
    // command.y_vel = 0.0;
    // command.w = 0.0;
    if (dribbler) {command.dribbler = true;}
    pub.publish(command);
    return Status::Running;
};

} // rtt
