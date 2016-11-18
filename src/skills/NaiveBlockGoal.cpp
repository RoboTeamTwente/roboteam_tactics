#include <cmath>

#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/NaiveBlockGoal.h"
#include "roboteam_tactics/skills/GoToPos.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

NaiveBlockGoal::NaiveBlockGoal(ros::NodeHandle n, std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(n, name, blackboard)
        , goToPos(n, "", private_bb) {
        	pubNaiveBlockGoal = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 1000);
            // ROS_INFO("NaiveBlockGoaling the ball");
}

bt::Node::Status NaiveBlockGoal::Update() {
    using namespace roboteam_utils;

    // std::cout << "\n\n\nUpdating nbg...\n";

    const double GOAL_AREA_WIDTH = 2.5 * 0.8;
    // Distance from front of goal area to goal
    const double GOAL_AREA_LENGTH = 1 * 0.8;
    const double FIELD_LENGTH = LastWorld::get_field().field_length;

    auto ballPos = Vector2(LastWorld::get().ball.pos);
    Vector2 goalPos;

    std::string our_field_side = "right";
    ros::param::get("our_field_side", our_field_side);
    if (our_field_side == "left") {
        goalPos = Vector2(FIELD_LENGTH / -2, 0);
    } else {
        goalPos = Vector2(FIELD_LENGTH / 2, 0);
    }

    auto ballVec = ballPos - goalPos;

    Vector2 horVec;
    Vector2 vertVec;

    vertVec = ballVec.normalize();
    vertVec = vertVec * std::abs(1 / vertVec.x);
    vertVec = vertVec * (GOAL_AREA_LENGTH);

    horVec = ballVec.normalize();
    horVec = horVec * std::abs(1 / horVec.y);
    horVec = horVec * (GOAL_AREA_WIDTH / 2);

    // TODO: Emit drawing commands?

    Vector2 minVec;

    if (!horVec.real()) {
        minVec = vertVec;
    } else if (!vertVec.real()) {
        minVec = horVec;
    } else if (vertVec.length() < horVec.length()) {
        minVec = vertVec;
    } else {
        minVec = horVec;
    }

    // std::cout << "Goal area width: " << GOAL_AREA_WIDTH << "\n";
    // std::cout << "Ball vec: " << ballVec.x  << " " << ballVec.y << "\n";
    // std::cout << "Hor vec: " << horVec.x << " " << horVec.y << "\n";
    // std::cout << "Vert vec: " << vertVec.x << " " << vertVec.y << "\n";
    // std::cout << "Min vec: " << minVec.x << " " << minVec.y << "\n";

    minVec = minVec + goalPos;
    // std::cout << "Target: " << minVec.x << " " << minVec.y << "\n";

    // minVec.x = 2.5;
    // minVec.y = 0;
    
    private_bb->SetInt("ROBOT_ID", blackboard->GetInt("ROBOT_ID"));
    if (minVec.real()) {
        private_bb->SetDouble("xGoal", minVec.x);
        private_bb->SetDouble("yGoal", minVec.y);
    }
    private_bb->SetDouble("angleGoal", ballVec.angle());
    private_bb->SetBool("endPoint", true);
    goToPos.Update();

    return Status::Running;
}

} // rtt
