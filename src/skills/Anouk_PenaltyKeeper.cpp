#include "roboteam_msgs/RoleDirective.h"

#include "roboteam_tactics/skills/Anouk_PenaltyKeeper.h"
#include "roboteam_tactics/treegen/LeafRegister.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/World.h"

#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"

#define ROS_LOG_NAME "skills.Anouk_PenaltyKeeper"

namespace rtt {

RTT_REGISTER_SKILL(Anouk_PenaltyKeeper);

Anouk_PenaltyKeeper::Anouk_PenaltyKeeper(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard), goToPos("", private_bb){}

void Anouk_PenaltyKeeper::publishStopCommand(int robotID) {
    roboteam_msgs::RobotCommand command;
    command.id = robotID;
    command.x_vel = 0.0;
    command.y_vel = 0.0;
    command.w = 0.0;
    command.dribbler = false;

    rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher().publish(command);
}

Vector2 Anouk_PenaltyKeeper::calculateKeeperPosition(){
    // Get the last world
    roboteam_msgs::World world = LastWorld::get();
    // Get the position of the ball
    Vector2 ballPos(world.ball.pos);
    // Get the position of the ball
    Vector2 goalPos(LastWorld::get_our_goal_center());


    // === Get their robot that is the closest to the ball === //
    int oppKicker = -1; // Holds the kicker ID
    double oppClosestDistance = 9999; // Holds the closest distance

    for(size_t i = 0; i < world.them.size(); i++){
        // Get the distance between the ball and the current opponent
        double distanceToBall = (Vector2(world.them.at(i).pos) - ballPos).length();
        // If this distance is closer than previous distances, store it
        if(distanceToBall < oppClosestDistance ){
            oppClosestDistance = distanceToBall;
            oppKicker = i;
        }
    }
    Vector2 oppKickerPos = Vector2(world.them.at(oppKicker).pos);
    // ======================================================= //

    // Angle between ball and kicker
    double angleBallOppKicker = (oppKickerPos - ballPos).angle();

    double y = tan(angleBallOppKicker) * -(ballPos.x-goalPos.x);
    double x = goalPos.x + 0.05;

    // === Don't let the keeper drive out of the goal === //
    // Half the width of the goal minus the diameter of the robot
    double maxOffset = 0.6 - 0.18;
    if(maxOffset < y ) y =  maxOffset;
    if(y < -maxOffset) y = -maxOffset ;

    return Vector2(x, y);
}

void Anouk_PenaltyKeeper::Initialize() {

    private_bb->SetInt("ROBOT_ID", blackboard->GetInt("ROBOT_ID"));
    private_bb->SetInt("KEEPER_ID", blackboard->GetInt("KEEPER_ID"));

}

bt::Node::Status Anouk_PenaltyKeeper::Update() {

    // Calculate the position of the keeper
    Vector2 keeperPos = Anouk_PenaltyKeeper::calculateKeeperPosition();
    // Pass the position to GoToPos
    private_bb->SetDouble("xGoal", keeperPos.x + 0.08);
    private_bb->SetDouble("yGoal", keeperPos.y);
    private_bb->SetDouble("angleGoal", 0);

    // Get the RobotCommand for the keeper and publish it
    boost::optional<roboteam_msgs::RobotCommand> commandPtr = goToPos.getVelCommand();
    if (commandPtr) {
        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        roboteam_msgs::RobotCommand command = *commandPtr;
        pub.publish(command);
    }else{
        Anouk_PenaltyKeeper::publishStopCommand(blackboard->GetInt("ROBOT_ID"));
    }

    // Keeper tactic is never done
    return bt::Node::Status::Running;
}


} // rtt

