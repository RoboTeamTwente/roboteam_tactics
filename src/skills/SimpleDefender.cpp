#include <string>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/SimpleDefender.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG SimpleDefender

namespace rtt {

RTT_REGISTER_SKILL(SimpleDefender);


SimpleDefender::SimpleDefender(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , receiveBall("", private_bb)
        , goToPos("", private_bb) { }

Vector2 SimpleDefender::computeDefensePoint(Vector2 defendPos, bool ourSide, double distanceFromGoal, double angleOffset) {
    
    Vector2 goalPos;
    // if (ourSide) {
        goalPos = LastWorld::get_our_goal_center();
    // } else {
        // goalPos = LastWorld::get_their_goal_center();
    // }
    
    double angle = (defendPos - goalPos).angle() + angleOffset;

    if (((defendPos - goalPos).length() - 0.5) < distanceFromGoal) {
        distanceFromGoal = (defendPos - goalPos).length() - 0.5;
    }

    Vector2 targetPos(distanceFromGoal, 0.0);
    targetPos = targetPos.rotate(angle);
    targetPos = goalPos + targetPos;

    return targetPos;
}

bt::Node::Status SimpleDefender::Update() {
    
    // Get the last world information and some blackboard info
    roboteam_msgs::World world = LastWorld::get();
    roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
    robotID = blackboard->GetInt("ROBOT_ID");

    bool ourSide;
    if (HasBool("ourSide")) {
        ourSide = GetBool("ourSide");
    } else {
        ourSide = true;
    }

    double acceptableDeviation;
    double dribblerDist;
    double distanceFromGoal;
    std::string fieldType = GetString("fieldType");
    if (fieldType == "office") {
        distanceFromGoal = 0.4;
        acceptableDeviation = 0.7;
        dribblerDist = 1.0;
    } else {
        distanceFromGoal = 0.7;
        acceptableDeviation = 0.8;
        dribblerDist = 2.0;
    }

    if (HasDouble("distanceFromGoal")) {
        distanceFromGoal = GetDouble("distanceFromGoal");
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
    Vector2 targetPos = computeDefensePoint(defendPos, ourSide, distanceFromGoal, angleOffset);


    ROS_INFO_STREAM("Robot: " << robotID << " targetPos: " << targetPos);

        
    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetInt("KEEPER_ID", GetInt("KEEPER_ID"));
    private_bb->SetDouble("receiveBallAtX", targetPos.x);
    private_bb->SetDouble("receiveBallAtY", targetPos.y);
    private_bb->SetDouble("acceptableDeviation", acceptableDeviation);
    private_bb->SetDouble("dribblerDist", dribblerDist);
    private_bb->SetBool("shouldFail", false);
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
    if (HasDouble("maxSpeed")) {
        private_bb->SetDouble("maxSpeed", GetDouble("maxSpeed"));
    }


    return receiveBall.Tick();
}

} // rtt