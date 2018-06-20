#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include "roboteam_tactics/conditions/CanInterceptBallDuel.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_tactics/utils/utils.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Vector2.h"


#include <string>

namespace rtt {

RTT_REGISTER_CONDITION(CanInterceptBallDuel);

CanInterceptBallDuel::CanInterceptBallDuel(std::string name, bt::Blackboard::Ptr blackboard)
        : Condition(name, blackboard) {}




bt::Node::Status CanInterceptBallDuel::Update() {
	const roboteam_msgs::World& world = LastWorld::get();

    // Get ID of closest robot to ball
    robotID = GetInt("me", blackboard->GetInt("ROBOT_ID"));


    // Find the robot with the specified ID
    boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(robotID);
    roboteam_msgs::WorldRobot robot;
    if (findBot) {
        robot = *findBot;
    } else {
        ROS_WARN_STREAM("BallPlacementTest: robot with this ID not found, ID: " << robotID);
        return Status::Failure;
    }


    // Get robot and ball position
	Vector2 ballPos(world.ball.pos);
    Vector2 robotPos(robot.pos);

    // Get ID of closest opponent to ball and its position
    int oppID = getClosestOppToPoint(ballPos, world);
    boost::optional<roboteam_msgs::WorldRobot> bot = getWorldBot(oppID, false, world);
    
    if(!bot){
        ROS_WARN_STREAM("Trying to defend bot that doesn't exist! : " << oppID);
        return Status::Failure;
    }

    Vector2 oppPos = (Vector2)bot->pos;

    // Find vector from robot to ball and opponent to ball
    Vector2 ballToRobot = (ballPos - robotPos);
    Vector2 ballToOpp = (ballPos - oppPos);

    // Calculate angle between upper two vectors
    double angleRobotOpp = (ballToRobot.angle() - ballToOpp.angle());

    // If opponent robot is in front of the ball the condition returns Failure, else the condition returns Success
    if (angleRobotOpp < -2.5 || angleRobotOpp > 2.5 ){
        return Status::Failure;
    }
    else {

        return Status::Success;
    }

}

} // rtt
