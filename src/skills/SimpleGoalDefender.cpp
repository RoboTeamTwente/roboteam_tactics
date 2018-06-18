#include <string>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/SimpleGoalDefender.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG SimpleGoalDefender

namespace rtt {

RTT_REGISTER_SKILL(SimpleGoalDefender);


SimpleGoalDefender::SimpleGoalDefender(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , receiveBall("", private_bb)
        , goToPos("", private_bb) { }

Vector2 SimpleGoalDefender::computeGoalDefensePoint(Vector2 defendPos, bool ourSide, double distanceFromGoal, double angleOffset) {

    // store location of the goal that matters
    Vector2 goalPos;
    if (ourSide) {
        goalPos = LastWorld::get_our_goal_center();
    } else {
        goalPos = LastWorld::get_their_goal_center();
    }

    // store angle of the vector between goal and defendPos(position which needs to be defended)
    double angle = (defendPos - goalPos).angle() + angleOffset;
//    std::cout << "SimpleGoalDefender angle = [" << angle << "] - [" << (M_1_PI * 2.5) << "] - [" << distanceFromGoal << "] - [" << (distanceFromGoal * tan(angle)) << "]" << std::endl;

    Vector2 targetPos;


    if (angle <= (M_1_PI * 2.5) && angle >= -(M_1_PI * 2.5)) {
        targetPos = Vector2(distanceFromGoal, (distanceFromGoal * tan(angle)));
        targetPos = goalPos + targetPos;
//        std::cout << "SimpleGoalDefender MID targetPos = [" << targetPos.x << ", " << targetPos.y << "]" << std::endl;
    } else if (angle > (M_1_PI * 2.5)){
        targetPos = Vector2(distanceFromGoal, (distanceFromGoal * tan((M_1_PI * 2.5) + angleOffset)));
        targetPos = goalPos + targetPos;
//        std::cout << "SimpleGoalDefender UP targetPos = [" << targetPos.x << ", " << targetPos.y << "]" << std::endl;
    } else if (angle < -(M_1_PI * 2.5)){
        targetPos = Vector2(distanceFromGoal, (distanceFromGoal * tan(-(M_1_PI * 2.5)+ angleOffset)));
        targetPos = goalPos + targetPos;
//        std::cout << "SimpleGoalDefender DOWN targetPos = [" << targetPos.x << ", " << targetPos.y << "]" << std::endl;
    } else {
//        std::cout << "SimpleGoalDefender angle = ERRORRRR" << std::endl;
    }
//    std::cout << "SimpleGoalDefender RETURN targetPos = [" << targetPos.x << ", " << targetPos.y << "]" << std::endl;
    return targetPos;

}


bt::Node::Status SimpleGoalDefender::Update() {
    
    // Get the last world information and some blackboard info
    roboteam_msgs::World world = LastWorld::get();
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    robotID = blackboard->GetInt("ROBOT_ID");

    bool ourSide = true;
    if (HasBool("ourSide")) {
        ourSide = GetBool("ourSide");
    }


    double acceptableDeviation;
    double dribblerDist;
    double distanceFromGoal;
    std::string fieldType = GetString("fieldType");
    if (fieldType == "office") {
        distanceFromGoal = 0.3;
        acceptableDeviation = 0.35;
        dribblerDist = 1.0;
    } else {
        distanceFromGoal = 0.7;
        acceptableDeviation = 0.2;
        dribblerDist = 2.0;
    }

    if (HasDouble("distanceFromGoal")) {
        distanceFromGoal = GetDouble("distanceFromGoal");
    }
    if (HasDouble("acceptableDeviation")) {
        acceptableDeviation = GetDouble("acceptableDeviation");
    }
    if (HasDouble("dribblerDist")) {
        dribblerDist = GetDouble("dribblerDist");
    }

    Vector2 defendPos;

    if (HasInt("defendRobot")) {
        unsigned int defendRobot = GetInt("defendRobot");
        boost::optional<roboteam_msgs::WorldRobot> findBot = getWorldBot(defendRobot, false, world);
        roboteam_msgs::WorldRobot robot;
        if (findBot) {
            robot = *findBot;
        } else {
            ROS_WARN_STREAM("SimpleKeeper: robot with this ID not found, ID: " << robotID);
            return Status::Failure;
        }  
        defendPos = Vector2(robot.pos);
    } else {
        defendPos = Vector2(world.ball.pos);
    }
    
    double angleOffset = GetDouble("angleOffset");
    Vector2 targetPos = computeGoalDefensePoint(defendPos, ourSide, distanceFromGoal, angleOffset);

        
    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
    private_bb->SetDouble("receiveBallAtX", targetPos.x);
    private_bb->SetDouble("receiveBallAtY", targetPos.y);
    private_bb->SetDouble("acceptableDeviation", acceptableDeviation);
    private_bb->SetDouble("dribblerDist", dribblerDist);
    private_bb->SetBool("defenderMode", true);
    private_bb->SetBool("dontDriveToBall", GetBool("dontDriveToBall"));
    private_bb->SetBool("setSignal", false);
    if (HasBool("avoidBallsFromOurRobots") && GetBool("avoidBallsFromOurRobots")) {
        private_bb->SetBool("avoidBallsFromOurRobots", true);
    }
    if (HasString("stayOnSide")) {
        private_bb->SetString("stayOnSide", GetString("stayOnSide"));
    }
    if (HasBool("stayAwayFromBall") && GetBool("stayAwayFromBall")) {
        private_bb->SetBool("stayAwayFromBall", true);
    }
    if (blackboard->HasDouble("maxSpeed")) {
        private_bb->SetDouble("maxSpeed", blackboard->GetDouble("maxSpeed"));
    }
    if (HasBool("enterDefenseAreas")) {
        private_bb->SetBool("enterDefenseAreas", GetBool("enterDefenseAreas"));
    }
    if (HasBool("pGainLarger")){
            private_bb->SetBool("pGainLarger", GetBool("pGainLarger"));
    }
    if (HasBool("avoidRobots")){
        private_bb->SetBool("avoidRobots", GetBool("avoidRobots"));
    }
    if (HasDouble("marginDeviation")) {
		private_bb->SetDouble("marginDeviation", GetDouble("marginDeviation"));
	}


    return receiveBall.Tick();
}

} // rtt
