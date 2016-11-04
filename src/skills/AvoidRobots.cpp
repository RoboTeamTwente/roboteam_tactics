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
#include "roboteam_utils/Vector2.h"

namespace rtt {

AvoidRobots::AvoidRobots(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard) {
	pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
}

bt::Node::Status AvoidRobots::Update (){

    // Set control gains
    double maxSpeed = 1.5;
    double attractiveForce = 10.0;
    double attractiveForceWhenClose = 4.0;
    double repulsiveForce = 10.0;


	roboteam_msgs::World world = LastWorld::get();
    if (world.us.size() == 0) {
        ROS_INFO("No information about the world state :(");
        return Status::Running;
    }

    double xGoal = GetDouble("xGoal");
    double yGoal = GetDouble("yGoal");
    double angleGoal = GetDouble("angleGoal");
    uint robotID = blackboard->GetInt("ROBOT_ID");
    bool dribbler = GetBool("dribbler");
    // bool priority = GetBool("priority");

    roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(xGoal, yGoal);
    roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us.at(robotID).pos.x, world.us.at(robotID).pos.y);
    roboteam_utils::Vector2 myVel = roboteam_utils::Vector2(world.us.at(robotID).vel.x, world.us.at(robotID).vel.y);
    roboteam_utils::Vector2 posError = targetPos - myPos;
    myVel.x = 1.0;
    myVel.y = 0.0;

    auto bb2 = std::make_shared<bt::Blackboard>();
    bb2->SetInt("me", robotID);
    bb2->SetDouble("x_coor", xGoal);
    bb2->SetDouble("y_coor", yGoal);

    // Set angleGoal such that you're always driving forwards, unless you're close to the target position
    if (posError.length() > 0.5) {
        angleGoal = posError.angle();
    }


    // Proportional rotation controller
    double requiredRotSpeed;
    double pGainRot = 6.0;
    double maxRotSpeed = 3.0;
    double angle = world.us.at(robotID).angle;
    double rotError = angleGoal - angle;

    if (rotError < M_PI) {rotError += 2*M_PI;}
    if (rotError > M_PI) {rotError -= 2*M_PI;}

    requiredRotSpeed = rotError * pGainRot;
    if (fabs(requiredRotSpeed) > maxRotSpeed) {
        requiredRotSpeed = requiredRotSpeed / fabs(requiredRotSpeed) * maxRotSpeed;
    }
    

    // If you can see the end point, just go towards it
    CanSeePoint canSeePoint("", bb2);
    if (canSeePoint.Update() == Status::Success) {
        roboteam_utils::Vector2 forceVector = posError*attractiveForceWhenClose;
        if (posError.length() > 0.3) {
            forceVector = forceVector.scale(1/forceVector.length() * maxSpeed);
        }

        // Rotate from robot frame to world frame
        roboteam_utils::Vector2 requiredSpeed;  
        requiredSpeed.x=forceVector.x*cos(-angle)-forceVector.y*sin(-angle);
        requiredSpeed.y=forceVector.x*sin(-angle)+forceVector.y*cos(-angle);

        if (posError.length() < 0.005 && rotError < 0.001) {
            roboteam_msgs::RobotCommand command;
            command.id = robotID;
            command.x_vel = 0.0;
            command.y_vel = 0.0;
            command.w = 0.0;
            if (dribbler) {command.dribbler = true;}
            pub.publish(command);
            ros::spinOnce();
            return Status::Success;
        } else {
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
        }
    }
    
    double lookingDistance = 0.75;
    if (lookingDistance > posError.length()) {
        lookingDistance = posError.length();
    }


    // Make a vector contaning the differences between other robots' positions and ours (if this 
    // distance is smaller than lookingDistance)
    std::vector<roboteam_utils::Vector2> positionDiffOtherRobots;
    double forceX2 = 0.0;
    double forceY2 = 0.0;

    for (size_t i = 0; i < world.us.size(); i++) {
        if (i != robotID) {
            roboteam_utils::Vector2 pos = roboteam_utils::Vector2(world.us.at(i).pos.x, world.us.at(i).pos.y);
            roboteam_utils::Vector2 posDiff = pos-myPos;
            roboteam_utils::Vector2 closestPointOnPath = posError.closestPointOnVector(myPos, pos);
            roboteam_utils::Vector2 distanceFromPath = pos - closestPointOnPath;
                        
            if (posDiff.length() < lookingDistance) {
                double scalingNumber1;
                if (distanceFromPath.length() < 0.05) {
                    scalingNumber1 = 1.0;
                } else if (distanceFromPath.length() > 0.2) {
                    scalingNumber1 = 0.0;
                } else {
                    scalingNumber1 = 1.0/(distanceFromPath.length()*20);
                }
                double scalingNumber2 = 1/(posDiff.length()*posDiff.length()*20);

                roboteam_utils::Vector2 distanceFromPathUnit = distanceFromPath.scale(1/distanceFromPath.length());
                roboteam_utils::Vector2 forceVector = distanceFromPathUnit.scale(scalingNumber1+scalingNumber2*2);
                forceX2 -= forceVector.x*repulsiveForce;
                forceY2 -= forceVector.y*repulsiveForce;
            } 
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_utils::Vector2 pos = roboteam_utils::Vector2(world.them.at(i).pos.x, world.them.at(i).pos.y);
        roboteam_utils::Vector2 posDiff = pos-myPos;
        roboteam_utils::Vector2 closestPointOnPath = posError.closestPointOnVector(myPos, pos);
        roboteam_utils::Vector2 distanceFromPath = pos - closestPointOnPath;
        // ROS_INFO_STREAM("distanceFromPath: " << distanceFromPath.length());
        
        if (posDiff.length() < lookingDistance) {
            double scalingNumber1;
            if (distanceFromPath.length() < 0.05) {
                scalingNumber1 = 1.0;
            } else if (distanceFromPath.length() > 0.2) {
                scalingNumber1 = 0.0;
            } else {
                scalingNumber1 = 1.0/(distanceFromPath.length()*20);
            }
            double scalingNumber2 = 1/(posDiff.length()*posDiff.length()*20);

            roboteam_utils::Vector2 distanceFromPathUnit = distanceFromPath.scale(1/distanceFromPath.length());
            roboteam_utils::Vector2 forceVector = distanceFromPathUnit.scale(scalingNumber1+scalingNumber2*2);
            forceX2 -= forceVector.x*repulsiveForce;
            forceY2 -= forceVector.y*repulsiveForce;
        }
    }

    forceX2 += posError.x*attractiveForce;
    forceY2 += posError.y*attractiveForce;

    ROS_INFO_STREAM("forceX2: " << forceX2 << " forceY2: " << forceY2);
    roboteam_utils::Vector2 forceVector = roboteam_utils::Vector2(forceX2, forceY2);

    if (posError.length() > 0.2) {
        if (forceVector.length() > 0) {
            forceVector = forceVector.scale(1/forceVector.length() * maxSpeed);
        } else {
            forceVector = roboteam_utils::Vector2(0.0, 0.0);
        }
    }


    // Rotate from robot frame to world frame
    roboteam_utils::Vector2 requiredSpeed;  
    requiredSpeed.x=forceVector.x*cos(-angle)-forceVector.y*sin(-angle);
    requiredSpeed.y=forceVector.x*sin(-angle)+forceVector.y*cos(-angle);

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
