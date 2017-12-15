#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/GenevaKick.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG GenevaKick

namespace rtt {

RTT_REGISTER_SKILL(GenevaKick);

GenevaKick::GenevaKick(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard), goneForward(false),
		  waitStart{time_point::min()}{
        cycleCounter = 0;
        }

void GenevaKick::Initialize() {
    auto vel = LastWorld::get().ball.vel;
    oldBallVel = Vector2(vel.x, vel.y);
    ballStartPos = LastWorld::get().ball.pos;
    cycleCounter = 0;
}

bt::Node::Status GenevaKick::Update() {

    cycleCounter++;
    if (cycleCounter > 20) {
        // ROS_INFO_STREAM("Kick Failure");
        return bt::Node::Status::Failure;
    }

	roboteam_msgs::World world = LastWorld::get();
    int robotID = blackboard->GetInt("ROBOT_ID");
    
    boost::optional<roboteam_msgs::WorldRobot> robotPointer = getWorldBot(robotID);
    roboteam_msgs::WorldRobot robot;
    if (robotPointer) {
        robot = *robotPointer;
    } else {
        ROS_WARN("Kick: Robot not found");
        return Status::Failure;
    }

//start turn geneva drive
    double genevaState;
    if (HasInt("genevaState")) {
        genevaState = GetInt("genevaState");
        std::cout << "HasInt succeeded" << std::endl;
    } else {
        genevaState = 0;    //default state
        std::cout << "HasInt failed" << std::endl;
    }

//start kicking process
    Vector2 robotDirVector = Vector2(1.0, 0.0).rotate(robot.angle);
    Vector2 currentBallVel(world.ball.vel);
    double currentBallVelInRobotDir = currentBallVel.dot(robotDirVector);
    double oldBallVelInRobotDir = oldBallVel.dot(robotDirVector);

        //test if the kick is successful. When successful return Success, this also means the code stops.
        if ((currentBallVelInRobotDir - oldBallVelInRobotDir) > 0.2 && currentBallVelInRobotDir >= 0.2) {
            return bt::Node::Status::Success;
        }


    oldBallVel = currentBallVel;

    
    //sets kick velocity
    double kickVel;
    if (HasDouble("kickVel")) {
    	kickVel = GetDouble("kickVel");     //given kick velocity
    } else {
    	kickVel = 10.0;     //default kick velocity
    }

    //This sets values for the robotcommand which is send to the robot
    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.geneva_state = genevaState;
    command.dribbler = true;
    command.kicker = true;
    command.kicker_forced = true;
    command.kicker_vel = kickVel;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;

    //publishes the values set above
    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
    std::cout << "current-old : "<<(currentBallVelInRobotDir - oldBallVelInRobotDir) << std::endl;
    std::cout << " current : " << currentBallVelInRobotDir << std::endl;

	// ROS_WARN_STREAM("Triggered the kicker!");
	return Status::Running;

}

} // rtt
