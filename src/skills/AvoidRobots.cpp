#include <string>

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
#include "roboteam_tactics/conditions/CanSeePoint.h"
#include <vector>

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
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
    double repulsiveForce = 1.0;


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

    roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(xGoal, yGoal);
    roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us[robotID].pos.x, world.us[robotID].pos.y);
    roboteam_utils::Vector2 myVel = roboteam_utils::Vector2(world.us[robotID].vel.x, world.us[robotID].vel.y);
    roboteam_utils::Vector2 posError = targetPos - myPos;

    auto bb2 = std::make_shared<bt::Blackboard>();
    bb2->SetInt("me", robotID);
    bb2->SetDouble("x_coor", xGoal);
    bb2->SetDouble("y_coor", yGoal);


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


        // Fill and send command
        roboteam_msgs::RobotCommand command;
        command.id = robotID;
        command.x_vel = requiredSpeed.x;
        command.y_vel = requiredSpeed.y;
        command.w = requiredRotSpeed;
        if (dribbler) {command.dribbler = true;}
        pub.publish(command);

        return Status::Running;
    }
    
    double lookingDistance = 1.5;
    if (lookingDistance > posError.length()) {
        lookingDistance = posError.length();
    }


    // Make a vector contaning the differences between other robots' positions and ours (if this 
    // distance is smaller than lookingDistance)
    std::vector<roboteam_utils::Vector2> positionDiffOtherRobots;
    for (size_t i = 0; i < world.us.size(); i++) {
        if (i != robotID) {
            roboteam_utils::Vector2 pos = roboteam_utils::Vector2(world.us[i].pos.x, world.us[i].pos.y);
            roboteam_utils::Vector2 posDiff = pos-myPos;
            posDiff = posDiff.scale((posDiff.length()-0.15)/posDiff.length());
            if (posDiff.length() < lookingDistance) {
                positionDiffOtherRobots.push_back(posDiff);
            }   
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_utils::Vector2 pos = roboteam_utils::Vector2(world.them[i].pos.x, world.them[i].pos.y);
        roboteam_utils::Vector2 posDiff = pos-myPos;
        posDiff = posDiff.scale((posDiff.length()-0.15)/posDiff.length());
        if (posDiff.length() < lookingDistance) {
            positionDiffOtherRobots.push_back(posDiff);
        }
    }


    // For each distance, compute a force the other robot exerts on our robot that is inversely
    // proportional to the square of the distance (just like gravity, but then repulsive instead
    // of attractive)
    double forceXMagn = 0.0;
    double forceYMagn = 0.0;
    if (positionDiffOtherRobots.size() > 0) {
        for (size_t i = 0; i < positionDiffOtherRobots.size(); i++) {
            double magnVector = positionDiffOtherRobots.at(i).length();
            roboteam_utils::Vector2 directionVector = positionDiffOtherRobots[i].scale(1/magnVector);
            double dot = directionVector.dot(posError.scale(1/posError.length()));
           
            double factor;
            if (dot > 0) {
                factor = dot*2*repulsiveForce;
            } else {
                factor = 0;
            }

            double forceMagn = factor/(magnVector*magnVector);
            forceXMagn -= forceMagn*directionVector.x;
            forceYMagn -= forceMagn*directionVector.y;
        }
    forceXMagn /= positionDiffOtherRobots.size();
    forceYMagn /= positionDiffOtherRobots.size();
    }

    roboteam_utils::Vector2 forceVector = roboteam_utils::Vector2(forceXMagn, forceYMagn);
    roboteam_utils::Vector2 loodrechteVector = posError.rotate(0.5*M_PI);
    double dot = forceVector.dot(loodrechteVector);
    roboteam_utils::Vector2 effectiveVector = loodrechteVector.scale(1/loodrechteVector.length() * dot);

    if (posError.length() > 0.3) {
        posError = posError.scale(1/posError.length());
    }
    if (effectiveVector.length() > 8.0) {
        effectiveVector = effectiveVector.scale(1/effectiveVector.length() * 8.0);
    }

    // Add another force that is an attractive force towards the target position
    effectiveVector.x += posError.x*attractiveForce;
    effectiveVector.y += posError.y*attractiveForce;
    forceVector = effectiveVector;

    roboteam_utils::Vector2 forceVectorUnit = forceVector.scale(1/forceVector.length());
    if (posError.length() > 0.2) {
        forceVector = forceVector.scale(1/forceVector.length() * maxSpeed);
    }

    // Rotate from robot frame to world frame
    roboteam_utils::Vector2 requiredSpeed;  
    requiredSpeed.x=forceVector.x*cos(-angle)-forceVector.y*sin(-angle);
    requiredSpeed.y=forceVector.x*sin(-angle)+forceVector.y*cos(-angle);


    // Fill and send command
    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.x_vel = requiredSpeed.x;
    command.y_vel = requiredSpeed.y;
    command.w = requiredRotSpeed;
    if (dribbler) {command.dribbler = true;}
    pub.publish(command);

    return Status::Running;
};

} // rtt