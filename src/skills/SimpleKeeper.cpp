#include <string>

#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_utils/LastWorld.h"
#include "roboteam_utils/Math.h"
#include "roboteam_utils/world_analysis.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/SimpleKeeper.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/utils.h"

#define RTT_CURRENT_DEBUG_TAG SimpleKeeper

namespace rtt {

RTT_REGISTER_SKILL(SimpleKeeper);


SimpleKeeper::SimpleKeeper(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , receiveBall("", private_bb)
        , goToPos("", private_bb) { }

Vector2 SimpleKeeper::computeDefensePoint(Vector2 defendPos, bool ourSide, double distanceFromGoal, double angleOffset) {
    
    Vector2 goalPos;
    if (ourSide) {
        goalPos = LastWorld::get_our_goal_center();
    } else {
        goalPos = LastWorld::get_their_goal_center();
    }
    
    double angle = (defendPos - goalPos).angle() + angleOffset;

    Vector2 targetPos(distanceFromGoal, 0.0);
    targetPos = targetPos.rotate(angle);
    targetPos = goalPos + targetPos;

    return targetPos;
}

bt::Node::Status SimpleKeeper::Update() {
    
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
        }  
        defendPos = Vector2(robot.pos);
    } else {
        defendPos = Vector2(world.ball.pos);
    }
    
    double angleOffset = GetDouble("angleOffset");
    Vector2 targetPos = computeDefensePoint(defendPos, ourSide, distanceFromGoal, angleOffset);

    // if (fabs(defendPos.x) > (field.field_length/2) || fabs(defendPos.y) > (field.field_width/2)) {
        // targetPos = goalPos - Vector2(distanceFromGoal, 0.0) * signum(goalPos.x);
        // private_bb->SetInt("ROBOT_ID", robotID);
        // private_bb->SetDouble("xGoal", targetPos.x);
        // private_bb->SetDouble("yGoal", targetPos.y);
        // private_bb->SetDouble("angleGoal", (Vector2(0.0, 0.0)-goalPos).angle());
        // private_bb->SetBool("avoidRobots", true);
        // goToPos.Update();
        // return Status::Running;
    // } else {   
        
        private_bb->SetInt("ROBOT_ID", robotID);
        private_bb->SetDouble("receiveBallAtX", targetPos.x);
        private_bb->SetDouble("receiveBallAtY", targetPos.y);
        private_bb->SetDouble("acceptableDeviation", acceptableDeviation);
        private_bb->SetDouble("dribblerDist", dribblerDist);

        private_bb->SetBool("dontDriveToBall", GetBool("dontDriveToBall"));

        return receiveBall.Tick();
    // }
}

} // rtt