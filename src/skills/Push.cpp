#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Push.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG Push

namespace rtt {

RTT_REGISTER_SKILL(Push);

Push::Push(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)

        {
        }


boost::optional<roboteam_msgs::RobotCommand> Push::getVelCommand() {
    ROBOT_ID = blackboard->GetInt("ROBOT_ID");
    // Fill the command message
    roboteam_msgs::RobotCommand command;
    command.id = ROBOT_ID;
    command.x_vel = 0.5;
    command.y_vel = 0.0;
    command.w = 0.0;
    command.dribbler = false;

    return command;
}

bt::Node::Status Push::Update() {

    boost::optional<roboteam_msgs::RobotCommand> command = getVelCommand();
    if (command) {
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        pub.publish(*command);
        return Status::Running;
    } else {
        //sendStopCommand(ROBOT_ID);
        //return Status::Success;
        //return Status::Failure;
        //return Status::Invalid;
    }
}

} // rtt

// Push::Push(std::string name, bt::Blackboard::Ptr blackboard)
//         : Skill(name, blackboard), goneForward(false),
// 		  waitStart{time_point::min()}{
//         cycleCounter = 0;
//         }
// void Push::Initialize() {
//     auto vel = LastWorld::get().ball.vel;
//     oldBallVel = Vector2(vel.x, vel.y);
//     ballStartPos = LastWorld::get().ball.pos;
//     cycleCounter = 0;
// }

// bt::Node::Status Push::Update() {

//     cycleCounter++;
//     if (cycleCounter > 20) {
//         // ROS_INFO_STREAM("Push Failure");
//         return bt::Node::Status::Failure;
//     }

// 	roboteam_msgs::World world = LastWorld::get();
//     int robotID = blackboard->GetInt("ROBOT_ID");
    
//     boost::optional<roboteam_msgs::WorldRobot> robotPointer = getWorldBot(robotID);
//     roboteam_msgs::WorldRobot robot;
//     if (robotPointer) {
//         robot = *robotPointer;
//     } else {
//         ROS_WARN("Push: Robot not found");
//         return Status::Failure;
//     }

//     Vector2 robotDirVector = Vector2(1.0, 0.0).rotate(robot.angle);
//     Vector2 currentBallVel(world.ball.vel);
//     double currentBallVelInRobotDir = currentBallVel.dot(robotDirVector);
//     double oldBallVelInRobotDir = oldBallVel.dot(robotDirVector);

//         if ((currentBallVelInRobotDir - oldBallVelInRobotDir) > 0.5 && currentBallVelInRobotDir >= 0.5) {
//             return bt::Node::Status::Success;
//         }


//     oldBallVel = currentBallVel;

    
    
//     double kickVel;
//     if (HasDouble("kickVel")) {
//     	kickVel = GetDouble("kickVel");
//     } else {
//     	kickVel = 5.0;
//     }
//     roboteam_msgs::RobotCommand command;
//     command.id = robotID;
//     command.dribbler = true;
//     command.kicker = true;
//     command.kicker_forced = true;
//     command.kicker_vel = kickVel;
//     command.x_vel = 0.0;
//     command.y_vel = 0.0;
//     command.w = 0.0;

//     rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
        
// 	// ROS_WARN_STREAM("Triggered the kicker!");
// 	return Status::Running;

// }

// void Kick::goForward() {
// 	static auto& pub = GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
// 	Vector2 forward(.5, 0);
// 	forward = forward.rotate(getWorldBot(robotID)->angle);
// 	roboteam_msgs::RobotCommand command;
// 	command.id = robotID;
// 	command.dribbler = true;
// 	command.kicker = false;
// 	command.kicker_forced = false;
// 	command.kicker_vel = 0.0;
// 	command.x_vel = forward.x;
// 	command.y_vel = forward.y;
// 	command.w = 0.0;
// 	pub.publish(command);
// 	goneForward = true;
// }

//} // rtt
