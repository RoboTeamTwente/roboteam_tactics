#include <string>

#include "ros/ros.h"
#include "roboteam_tactics/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/AvoidRobots.h"
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
	roboteam_msgs::World world = LastWorld::get();

    // ROS_INFO_STREAM("us: " << world.us.size() << " them: " << world.them.size());
    if (world.us.size() == 0) {
        ROS_INFO("No information about the world state :(");
        return Status::Running;
    }

    size_t robotID = blackboard->GetInt("ROBOT_ID");
    double targetX = GetDouble("targetX");
    double targetY = GetDouble("targetY");
    roboteam_utils::Vector2 targetPos = roboteam_utils::Vector2(targetX, targetY);

    roboteam_utils::Vector2 myPos = roboteam_utils::Vector2(world.us[robotID].pos.x, world.us[robotID].pos.y);
    roboteam_utils::Vector2 myVel = roboteam_utils::Vector2(world.us[robotID].vel.x, world.us[robotID].vel.y);
    roboteam_utils::Vector2 posError = targetPos - myPos;

    // ROS_INFO_STREAM("myVel x: " << myVel.x << " myVel y: " << myVel.y);

    std::vector<roboteam_utils::Vector2> positionsOtherRobots;
    
    // double lookingDistance = myVel.length();
    double lookingDistance = 1.0;
    if (lookingDistance > posError.length()) {
        lookingDistance = posError.length();
    }

    // ROS_INFO_STREAM(lookingDistance);

    for (size_t i = 0; i < world.us.size(); i++) {
        if (i != robotID) {
            roboteam_utils::Vector2 pos = roboteam_utils::Vector2(world.us[i].pos.x, world.us[i].pos.y);
            // ROS_INFO_STREAM("dist to robot " << i << " of our team: " << (pos-myPos).x << " " << (pos-myPos).y);
            roboteam_utils::Vector2 posDiff = pos-myPos;
            if (posDiff.length() < lookingDistance) {
                positionsOtherRobots.push_back(pos-myPos);
            }
            
        }
    }
    for (size_t i = 0; i < world.them.size(); i++) {
        roboteam_utils::Vector2 pos = roboteam_utils::Vector2(world.them[i].pos.x, world.them[i].pos.y);
        // ROS_INFO_STREAM("dist to robot " << i << " of their team: " << (pos-myPos).x << " " << (pos-myPos).y);
        roboteam_utils::Vector2 posDiff = pos-myPos;
            if (posDiff.length() < lookingDistance) {
                positionsOtherRobots.push_back(pos-myPos);
            }
    }

    double forceXMagn = 0.0;
    double forceYMagn = 0.0;

    if (positionsOtherRobots.size() > 0) {
        for (size_t i = 0; i < positionsOtherRobots.size(); i++) {
            double magnVector = positionsOtherRobots.at(i).length();
            roboteam_utils::Vector2 directionVector = positionsOtherRobots[i].scale(1/magnVector);
            double dot = directionVector.dot(myVel);
            double factor;
            // ROS_INFO_STREAM("id: " << i << " dot: " << dot);
            if (fabs(dot) < 0.1) {dot = dot / dot * 0.1;}
            if (dot > 0) {
                factor = 0.3 + dot*2;
            } else {
                factor = 0.3;
            }

            // ROS_INFO_STREAM("id: " << i << " factor: " << factor);
            double forceMagn = factor/(magnVector*magnVector);
            forceXMagn -= forceMagn*directionVector.x;
            forceYMagn -= forceMagn*directionVector.y;
        }
    forceXMagn /= positionsOtherRobots.size();
    forceYMagn /= positionsOtherRobots.size();
    }

    if (posError.length() > 2) {
        posError = posError.scale(2/posError.length());
    }

    forceXMagn += posError.x*2;
    forceYMagn += posError.y*2;

    // Proportional rotation controller
    double requiredRotSpeed;
    double pGainRot = 6.0;
    double maxRotSpeed = 3.0;
    double angle = world.us.at(robotID).angle;
    double rotError = 0.0 - angle;

    if (rotError < M_PI) {rotError += 2*M_PI;}
    if (rotError > M_PI) {rotError -= 2*M_PI;}

    requiredRotSpeed = rotError * pGainRot;
    if (fabs(requiredRotSpeed) > maxRotSpeed) {
        requiredRotSpeed = requiredRotSpeed / fabs(requiredRotSpeed) * maxRotSpeed;
    }

    roboteam_utils::Vector2 forceVector = roboteam_utils::Vector2(forceXMagn, forceYMagn);
    roboteam_utils::Vector2 forceVectorUnit = forceVector.scale(1/forceVector.length());

    if (posError.length() > 0.3) {
        forceVector = forceVector.scale(1/forceVector.length() * 1);
    }

    // Rotate from robot frame to world frame
    roboteam_utils::Vector2 requiredSpeed;  
    
    requiredSpeed.x=forceXMagn*cos(angle)-forceYMagn*sin(angle);
    requiredSpeed.y=forceXMagn*sin(angle)+forceYMagn*cos(angle);
    // ROS_INFO_STREAM("X force: " << forceXMagn << ", Y force: " << forceYMagn);

    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.x_vel = forceVector.x;
    command.y_vel = forceVector.y;
    command.w = requiredRotSpeed;
    pub.publish(command);

    return Status::Running;
};

} // rtt