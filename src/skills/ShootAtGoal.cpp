#include "roboteam_tactics/treegen/LeafRegister.h"
#include "ros/ros.h"
#include "roboteam_tactics/utils/LastWorld.h"
#include "roboteam_tactics/utils/Math.h"

#include "roboteam_tactics/Parts.h"
#include "roboteam_tactics/skills/Kick.h"
#include "roboteam_tactics/skills/ShootAtGoal.h"
#include "roboteam_tactics/utils/debug_print.h"
#include "roboteam_tactics/utils/Math.h"
#include "roboteam_tactics/utils/Cone.h"

#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/RobotCommand.h"

#include "roboteam_utils/Vector2.h"



#define RTT_CURRENT_DEBUG_TAG ShootAtGoal

namespace rtt {

RTT_REGISTER_SKILL(ShootAtGoal);

ShootAtGoal::ShootAtGoal(std::string name, bt::Blackboard::Ptr blackboard)
        : Skill(name, blackboard)
        , rotateBB(std::make_shared<bt::Blackboard>())
        , kickBB(std::make_shared<bt::Blackboard>())
        , rotateAroundPoint("", rotateBB) 
        , kick("", kickBB) { 

	robotID = blackboard->GetInt("ROBOT_ID");
	rotateBB->SetInt("ROBOT_ID", robotID);
    rotateBB->SetString("center", "ball");
    rotateBB->SetDouble("w",3.0);
    rotateBB->SetDouble("radius", 0.1);
    kickBB->SetInt("ROBOT_ID", robotID);
}

roboteam_utils::Vector2 ShootAtGoal::DetermineTarget() {
	roboteam_msgs::World world = LastWorld::get();
	roboteam_msgs::GeometryFieldSize field = LastWorld::get_field();
	roboteam_utils::Vector2 theirGoalCenter = LastWorld::get_their_goal_center();
	double goalWidth = field.goal_width;
	roboteam_utils::Vector2 theirGoalLeft = Vector2(theirGoalCenter.x, theirGoalCenter.y + 0.5*goalWidth*signum(theirGoalCenter.x));
	roboteam_utils::Vector2 theirGoalRight = Vector2(theirGoalCenter.x, theirGoalCenter.y - 0.5*goalWidth*signum(theirGoalCenter.x));

	roboteam_utils::Vector2 meToGoalLeft = theirGoalLeft - myPos;
	roboteam_utils::Vector2 meToGoalRight = theirGoalRight - myPos;
	Cone goalCone(myPos, meToGoalLeft, meToGoalRight);

	drawer.SetColor(255, 0, 0);
	drawer.DrawLine("side1", goalCone.start, goalCone.side1);
	drawer.SetColor(0, 0, 0);
	drawer.DrawLine("side2", goalCone.start, goalCone.side2);
	drawer.DrawPoint("center", goalCone.center);

	std::vector<roboteam_msgs::WorldRobot> watchOutForTheseBots;

	for (size_t i = 0; i < world.us.size(); i++) {
		boost::optional<roboteam_msgs::WorldRobot> bot = lookup_bot(i, true, &world);
		roboteam_msgs::WorldRobot currentRobot;
		if (bot) {
			currentRobot = *bot;
		} else {
			currentRobot = me;
		}
		
		if (currentRobot.id != me.id) {
			if (goalCone.IsWithinCone(Vector2(currentRobot.pos))) {
				watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
			}  
		}
    }

	for (size_t i = 0; i < world.them.size(); i++) {
		roboteam_msgs::WorldRobot currentRobot = world.them.at(i);
		if (goalCone.IsWithinCone(Vector2(currentRobot.pos))) {
			watchOutForTheseBots.insert(watchOutForTheseBots.end(), currentRobot);
		}  
    }

    roboteam_utils::Vector2 targetPos;
    if (watchOutForTheseBots.size() > 1) {
    	ROS_INFO("more than one robot blocking the goal, I don't know what to do aaaah");
    	targetPos = theirGoalCenter;
    } else if (watchOutForTheseBots.size() == 0) {q
    	targetPos = theirGoalCenter;
    } else {
    	roboteam_utils::Vector2 keeperPos = Vector2(watchOutForTheseBots.at(0).pos);
    	Cone keeperCone(myPos, keeperPos, 0.09);

    	drawer.SetColor(255, 0, 0);
    	drawer.DrawLine("keeperside1", keeperCone.start, keeperCone.side1);
    	drawer.SetColor(0, 0, 0);
		drawer.DrawLine("keeperside2", keeperCone.start, keeperCone.side2);
		drawer.DrawPoint("keepercenter", keeperCone.center);


    	if (fabs(goalCone.side1.angle() - keeperCone.side1.angle()) >= fabs(goalCone.side2.angle() - keeperCone.side2.angle())) {
    		// shoot to the left corner of the goal
    		targetPos.x = theirGoalCenter.x;
    		targetPos.y = goalWidth * 0.4 * signum(theirGoalCenter.x);
    	} else {
    		// shoot to the right corner of the goal
    		targetPos.x = theirGoalCenter.x;
    		targetPos.y = goalWidth * -0.4 * signum(theirGoalCenter.x);
    	}
    }
    return targetPos;
}

bt::Node::Status ShootAtGoal::Update() {
	roboteam_msgs::World world = LastWorld::get();
	boost::optional<roboteam_msgs::WorldRobot> bot = lookup_bot(robotID, true, &world);
	
	if (bot) {
		me = *bot;
	} else {
		ROS_WARN("ShootAtGoal: robot not found");
		return Status::Failure;
	}

	double kickVel;
	if (HasDouble("kickVel")) {
		kickVel = GetDouble("kickVel");
	} else {
		kickVel = 4.0;
	}

	myPos = Vector2(me.pos);

	roboteam_utils::Vector2 targetPos = DetermineTarget();

    rotateBB->SetDouble("faceTowardsPosx", targetPos.x);
    rotateBB->SetDouble("faceTowardsPosy", targetPos.y);
    
    if (resultRotate != Status::Success) {
    	resultRotate = rotateAroundPoint.Update();
    }

    if (resultRotate == Status::Success) {
    	if (!initializedKick) {
    		kick.Initialize();
    		initializedKick = true;
    	}
    	kickBB->SetDouble("kickVel", kickVel);
    	Status resultKick = kick.Update();
    	if (resultKick == Status::Success) {
    		drawer.RemovePoint("center");
    		drawer.RemoveLine("side1");
    		drawer.RemoveLine("side2");
    		drawer.RemovePoint("keepercenter");
    		drawer.RemoveLine("keeperside1");
    		drawer.RemoveLine("keeperside2");
    		return Status::Success;
    	}
    } else {

    }

	return Status::Running;
}

} // rtt
